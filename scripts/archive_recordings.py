#!/usr/bin/env python3
"""Archive closed Jetson recording batches to a mounted disk, verify, then clean up.

Host GUI: python3 archive_recordings.py --ssh openarm-jetson
Jetson CLI: python3 archive_recordings.py --session /home/nvidia/datasets/BATCH --disk /media/nvidia/DISK
The per-batch number ledger remains on Jetson; episode names are never changed.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import shlex
import shutil
import subprocess
import sys
import uuid

DATA_ROOT = Path('/home/nvidia/datasets')
REMOTE_SCRIPT = '/home/nvidia/dev/openarm-rgbd-preview/scripts/archive_recordings.py'
CHUNK = 8 * 1024 * 1024


def write_json(path, value):
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('w', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2)
        f.flush(); os.fsync(f.fileno())
    tmp.replace(path)


def recorder_state():
    import zmq
    context = zmq.Context()
    sock = context.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, 3000)
    sock.setsockopt(zmq.SNDTIMEO, 3000)
    try:
        sock.connect('tcp://127.0.0.1:5557')
        sock.send_json({'command':'status'})
        value = sock.recv_json()
        if not value.get('ok'): raise RuntimeError('无法确认采集服务状态')
        return value
    finally:
        sock.close(); context.term()


def require_idle(source):
    state = recorder_state()
    if state.get('running') or state.get('active_episode'):
        raise RuntimeError('请先停止录制、标记结果并结束采集批次，再转存。')
    if state.get('session_root') == str(source):
        raise RuntimeError('这个批次仍处于打开状态，请先在采集控制台结束批次。')


def mounted_disks():
    result = subprocess.run(['lsblk','--json','-o','NAME,TRAN,FSTYPE,LABEL,MOUNTPOINTS'],
                            capture_output=True,text=True,check=True)
    disks = []
    def visit(node, transport=None):
        transport = node.get('tran') or transport
        if transport == 'usb':
            for mount in node.get('mountpoints') or []:
                if mount and Path(mount).is_dir() and os.path.ismount(mount):
                    free = shutil.disk_usage(mount).free
                    disks.append({'path':mount,'label':node.get('label') or node['name'],
                                  'free_gib':round(free/1024**3,1)})
        for child in node.get('children',[]): visit(child,transport)
    for node in json.loads(result.stdout)['blockdevices']: visit(node)
    return disks


def check_disk(disk, source, expected_device=None):
    if disk.is_symlink() or not disk.is_dir() or not os.path.ismount(disk):
        raise ValueError('请选择已挂载移动硬盘的根目录，不能使用普通文件夹。')
    device = disk.stat().st_dev
    if device == source.stat().st_dev:
        raise ValueError('目的地与原始数据在同一个分区，无法释放 Jetson 空间。')
    if expected_device is not None and device != expected_device:
        raise RuntimeError('移动硬盘挂载发生变化，保留 Jetson 原件。')
    if not os.access(disk,os.W_OK): raise ValueError('移动硬盘不可写。')
    return device


def fingerprint(path):
    s = path.stat()
    return (s.st_size,s.st_mtime_ns,s.st_ctime_ns,s.st_ino,s.st_dev)


def inventory(root):
    files = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink(): raise ValueError('数据包含符号链接，停止转存：'+str(path))
        if path.name == '.archive.lock': continue
        if path.is_file(): files[path.relative_to(root).as_posix()] = fingerprint(path)
        elif not path.is_dir(): raise ValueError('数据中有不支持的特殊文件：'+str(path))
    return files


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(CHUNK),b''): h.update(block)
    return h.hexdigest()


def list_recordings():
    batches = []
    for manifest in DATA_ROOT.rglob('session.json'):
        try:
            value = json.loads(manifest.read_text())
            episodes = list((manifest.parent/'episodes').glob('*/episode_*'))
            if not episodes: continue
            batches.append({'path':str(manifest.parent),'count':len(episodes),
                'closed':value.get('session_ended_unix_s') is not None,
                'label':f"{manifest.parent.name} · {len(episodes)} 条 · "+
                    ('已结束' if value.get('session_ended_unix_s') is not None else '需先结束批次')})
        except (OSError,ValueError): continue
    return {'batches':batches,'disks':mounted_disks()}


def archive_batch(source, disk, progress=lambda message:None):
    source, disk = Path(source).absolute(), Path(disk).absolute()
    if source.is_symlink() or source.resolve() != source or DATA_ROOT.resolve() not in source.parents:
        raise ValueError('只允许转存 Jetson datasets 内的真实批次目录。')
    require_idle(source)
    device = check_disk(disk,source)
    with (source/'.archive.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        meta = json.loads((source/'session.json').read_text())
        if meta.get('session_ended_unix_s') is None:
            raise ValueError('批次尚未结束，请先在控制台结束批次。')
        if list(source.rglob('episode.pending.json')) or list(source.glob('.archive-cleanup-*')):
            raise ValueError('存在未完成录制或上次清理残留，请先检查恢复。')
        episodes = sorted((source/'episodes').glob('*/episode_*'))
        if not episodes: raise ValueError('本批次没有可转存的录制。')
        for episode in episodes:
            value = json.loads((episode/'episode.json').read_text())
            if value.get('result') not in ('success','failure','aborted'):
                raise ValueError('存在尚未标记结果的录制：'+str(episode))
            receipt = json.loads((episode/'lerobot/rgbd-complete.json').read_text())
            if receipt.get('complete') is not True:
                raise ValueError('存在未写完的数据：'+str(episode))
        # Initialize the durable ledger for old, already closed batches.
        ledger_path = source/'.episode-sequence.json'
        ledger = json.loads(ledger_path.read_text()) if ledger_path.exists() else {
            'schema_version':1,'batch_uid':uuid.uuid4().hex,'next_episode_by_task':{}}
        if (not isinstance(ledger.get('batch_uid'),str) or len(ledger['batch_uid'])!=32
            or any(c not in '0123456789abcdef' for c in ledger['batch_uid'])
            or not isinstance(ledger.get('next_episode_by_task'),dict)
            or any(not g or not g.replace('_','').isalnum() for g in ledger['next_episode_by_task'])
            or any(type(n) is not int or n<1 for n in ledger['next_episode_by_task'].values())):
            raise ValueError('批次编号账本损坏')
        for episode in episodes:
            group, number = episode.parent.name,int(episode.name.removeprefix('episode_'))
            if not group.replace('_','').isalnum() or number<1: raise ValueError('条目编号无效')
            ledger['next_episode_by_task'][group] = max(ledger['next_episode_by_task'].get(group,1),number+1)
        write_json(ledger_path,ledger)
        before = inventory(source)
        total = sum(info[0] for info in before.values())
        if shutil.disk_usage(disk).free < total + 1024**3:
            raise ValueError('移动硬盘剩余空间不足，需要容纳本批次并保留 1 GiB 余量。')
        identity = datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        parent = disk/'OpenArm采集归档'/ledger['batch_uid']
        parent.mkdir(parents=True,exist_ok=True)
        if parent.resolve()!=parent or parent.stat().st_dev!=device:
            raise ValueError('归档路径不是所选硬盘上的真实目录。')
        staging, output = parent/('.partial-'+identity),parent/identity
        staging.mkdir()
        copied = 0; hashes = {}
        try:
            for name, info in before.items():
                check_disk(disk,source,device)
                src, dst = source/name,staging/name
                dst.parent.mkdir(parents=True,exist_ok=True)
                h = hashlib.sha256()
                with src.open('rb') as inp, dst.open('xb') as out:
                    for block in iter(lambda:inp.read(CHUNK),b''):
                        out.write(block);h.update(block);copied+=len(block)
                        if copied//(128*1024**2) != (copied-len(block))//(128*1024**2):
                            progress(f'复制 {copied/total:.0%}，正在处理 {name}')
                    out.flush();os.fsync(out.fileno())
                if fingerprint(src)!=info: raise RuntimeError('原始文件发生变化，保留全部原件。')
                progress('校验 SHA-256：'+name)
                if digest(dst)!=h.hexdigest(): raise RuntimeError('硬盘文件校验失败，保留全部原件。')
                hashes[name]={'bytes':info[0],'sha256':h.hexdigest()}
            if inventory(source)!=before: raise RuntimeError('批次内容发生变化，保留全部原件。')
            require_idle(source);check_disk(disk,source,device)
            receipt={'schema_version':1,'batch_uid':ledger['batch_uid'],'source':str(source),
                'archive':str(output),'episodes':[p.relative_to(source).as_posix() for p in episodes],
                'files':hashes,'bytes':total,'verification':'SHA-256 of every copied file',
                'next_episode_by_task':ledger['next_episode_by_task']}
            write_json(staging/'archive-manifest.json',receipt)
            staging.rename(output)
            # Flush the destination filesystem before removing any source data.
            subprocess.run(['sync','-f',str(output)],check=True)
        except BaseException:
            # A partial copy is retained for inspection; never mistaken for a completed archive.
            raise
        detached = source/('.archive-cleanup-'+identity)
        try:
            require_idle(source);check_disk(disk,source,device)
            if inventory(source)!=before: raise RuntimeError('源批次改变，已校验的硬盘副本保留，原件不删除。')
            (source/'episodes').rename(detached)
            # The manager serializes session selection and status. With episodes absent,
            # a new selection cannot open this batch; a selection already in flight is detected here.
            require_idle(source)
            expected = {n[len('episodes/'):]:v for n,v in before.items() if n.startswith('episodes/')}
            if inventory(detached)!=expected: raise RuntimeError('清理前检测到数据变化，恢复原件。')
            check_disk(disk,source,device)
            if not (output/'archive-manifest.json').is_file(): raise RuntimeError('硬盘归档已不可访问')
            meta['episodes']=[]
            meta.setdefault('archives',[]).append({k:receipt[k] for k in ['archive','episodes','bytes']})
            write_json(source/'session.json',meta)
            shutil.rmtree(detached)
            (source/'episodes').mkdir()
            for group in ledger['next_episode_by_task']: (source/'episodes'/group).mkdir(exist_ok=True)
        except BaseException:
            if detached.exists() and not (source/'episodes').exists(): detached.rename(source/'episodes')
            raise
        return {'archive':str(output),'episodes':len(episodes),'bytes':total,
                'next_episode_by_task':ledger['next_episode_by_task']}


def remote_request(host, request, progress):
    command=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10',
             '-o','ServerAliveCountMax=3',host,
             shlex.join(['nice','-n','10','taskset','-c','3-7','/usr/bin/python3','-u',REMOTE_SCRIPT,'--worker'])]
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    process.stdin.write(json.dumps(request)+'\n');process.stdin.close()
    result=None
    for line in process.stdout:
        value=json.loads(line)
        if 'progress' in value:progress(value['progress'])
        else:result=value
    error=process.stderr.read()
    if process.wait() or not result or not result.get('ok'):
        raise RuntimeError((result or {}).get('error') or error[-1200:] or 'Jetson 转存连接中断')
    return result['result']


def show_gui(host):
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('采集数据转存到 Jetson 移动硬盘');root.geometry('930x410')
    frame=ttk.Frame(root,padding=20);frame.pack(fill='both',expand=True)
    ttk.Label(frame,text='复制 → 逐文件 SHA-256 校验 → 清理 Jetson 原件',font=('',14)).pack(anchor='w',pady=10)
    ttk.Label(frame,text='请先在采集控制台结束批次。转存期间不要重新打开该批次或拔出硬盘。').pack(anchor='w')
    ttk.Label(frame,text='已录制批次').pack(anchor='w',pady=(15,0));batch=ttk.Combobox(frame,state='readonly');batch.pack(fill='x')
    ttk.Label(frame,text='Jetson 上已挂载的 USB 移动硬盘').pack(anchor='w',pady=(12,0));disk=ttk.Combobox(frame,state='readonly');disk.pack(fill='x')
    message=tk.StringVar(value='正在读取 Jetson…');ttk.Label(frame,textvariable=message,wraplength=880).pack(anchor='w',pady=15)
    ttk.Label(frame,text='原编号保留；删除后不回收编号。硬盘上保存完整录制和校验清单。').pack(anchor='w')
    pool=ThreadPoolExecutor(max_workers=1);messages=queue.Queue();state={'future':None,'catalog':{},'kind':None}
    def submit(kind,request):
        if state['future'] is not None:return
        state['kind']=kind;state['future']=pool.submit(remote_request,host,request,messages.put)
        start.state(['disabled']);refresh.state(['disabled'])
    def load():submit('list',{'command':'list'})
    def archive():
        if batch.current()<0 or disk.current()<0:
            message.set('请选择批次和硬盘；未检测到硬盘时，请接入并挂载后刷新。');return
        entry=state['catalog']['batches'][batch.current()]
        if not entry['closed']:message.set('请先在采集控制台结束这个批次，再刷新。');return
        message.set('正在转存；校验成功后会清理本批次的 Jetson 原件…')
        submit('archive',{'command':'archive','source':entry['path'],'disk':state['catalog']['disks'][disk.current()]['path']})
    row=ttk.Frame(frame);row.pack(fill='x',pady=12)
    start=ttk.Button(row,text='转存、校验并清理原件',command=archive);start.pack(side='left')
    refresh=ttk.Button(row,text='刷新批次和硬盘',command=load);refresh.pack(side='left',padx=12)
    def tick():
        while not messages.empty():message.set(messages.get_nowait())
        future=state['future']
        if future is not None and future.done():
            kind=state['kind'];state['future']=None;start.state(['!disabled']);refresh.state(['!disabled'])
            try:
                result=future.result()
                if kind=='list':
                    state['catalog']=result
                    batch.configure(values=[r['label'] for r in result['batches']])
                    disk.configure(values=[f"{d['label']} · {d['path']} · 剩余 {d['free_gib']} GiB" for d in result['disks']])
                    batch.set('');disk.set('')
                    if result['batches']:batch.current(0)
                    if result['disks']:disk.current(0)
                    message.set(f"找到 {len(result['batches'])} 个有数据的批次、{len(result['disks'])} 块已挂载移动硬盘。")
                else:
                    batch.set('');batch.configure(values=[])
                    message.set(f"完成：{result['episodes']} 条，{result['bytes']/1024**3:.2f} GiB。原件已清理，历史编号保留。\n"+result['archive'])
            except Exception as exc:message.set('未完成：'+str(exc))
        root.after(150,tick)
    def close():
        if state['future'] is not None:message.set('正在处理，请等待完成后关闭。');return
        pool.shutdown(wait=False);root.destroy()
    root.protocol('WM_DELETE_WINDOW',close);load();root.after(150,tick);root.mainloop()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ssh',default='openarm-jetson');parser.add_argument('--worker',action='store_true')
    parser.add_argument('--session');parser.add_argument('--disk')
    args=parser.parse_args()
    if args.worker:
        try:
            request=json.loads(sys.stdin.readline())
            if request.get('command')=='list':result=list_recordings()
            elif request.get('command')=='archive':
                result=archive_batch(request['source'],request['disk'],lambda m:print(json.dumps({'progress':m},ensure_ascii=False),flush=True))
            else:raise ValueError('未知命令')
            print(json.dumps({'ok':True,'result':result},ensure_ascii=False),flush=True)
        except Exception as exc:
            print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False),flush=True);raise SystemExit(1)
    elif args.session:
        if not args.disk:parser.error('--session 必须同时指定 --disk')
        print(json.dumps(archive_batch(args.session,args.disk,print),ensure_ascii=False,indent=2))
    else:
        if args.ssh.startswith('-'):parser.error('SSH 主机无效')
        show_gui(args.ssh)


if __name__=='__main__':main()
