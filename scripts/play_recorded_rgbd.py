#!/usr/bin/env python3
"""Play recorded RGB + depth as a normal seekable video, without robot access.

  python3 play_recorded_rgbd.py --ssh openarm-jetson
  python3 play_recorded_rgbd.py --ssh openarm-jetson /remote/episode/path

Creates a disposable WebM preview in ~/.cache/openarm-rgbd-player. Source
RGB/depth remain lossless and untouched. Requires the companion raw-frame
viewer, numpy/Pillow, ffmpeg on the data host and Totem on the viewing host.
"""
from __future__ import annotations
import argparse
from bisect import bisect_left
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import queue
import shlex
import shutil
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw
from view_recorded_rgbd import Recording, Transport, depth_colors

VERSION = 'rgbd-video-v1'
CACHE = Path.home()/'.cache/openarm-rgbd-player'


def source_signature(recording, near, far):
    files = [recording.root/'rgbd-complete.json']
    for role in recording.cameras:
        files += [recording.root/f'depth_raw/{role}.jsonl', recording.root/f'depth_raw/{role}.u16le',
                  recording.root/f'rgb_raw/{role}.rgb24']
    return hashlib.sha256(json.dumps([VERSION, str(recording.root), near, far,
        [(str(p), p.stat().st_size, p.stat().st_mtime_ns) for p in files]], sort_keys=True).encode()).hexdigest()


def render_video(path, near=.1, far=3., progress=lambda message: None):
    recording = Recording(path)
    receipt = json.loads((recording.root/'rgbd-complete.json').read_text())
    if receipt.get('complete') is not True:
        raise ValueError('录制尚未保存完成，请先保存本条数据。')
    depth_colors(np.zeros((1, 1), dtype=np.uint16), 1., near, far)
    if recording.warnings: raise ValueError('录制索引异常：'+'；'.join(recording.warnings))
    key = source_signature(recording, near, far)
    CACHE.mkdir(parents=True, exist_ok=True)
    output = CACHE/(key+'.webm')
    if output.exists() and output.stat().st_size > 0:
        return {'path': str(output), 'filename': output.name, 'cached': True, 'duration': recording.duration}
    encoder = shutil.which('ffmpeg')
    if not encoder: raise RuntimeError('数据所在电脑缺少 ffmpeg，无法生成视频预览。')
    roles = [r for r in ('left_wrist', 'right_wrist', 'chest') if r in recording.cameras]
    roles += [r for r in recording.cameras if r not in roles]
    width, height = 320*len(roles), 480
    count = max(1, math.ceil(recording.duration*30)+1)
    with tempfile.TemporaryDirectory(prefix='render-', dir=CACHE) as tmp:
        target = Path(tmp)/'preview.webm'
        with (Path(tmp)/'ffmpeg.log').open('w+') as errors:
            command = [encoder, '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                '-s', f'{width}x{height}', '-r', '30', '-i', 'pipe:0', '-an', '-c:v', 'libvpx',
                '-threads', '2', '-deadline', 'realtime', '-cpu-used', '5', '-b:v', '2500k',
                '-pix_fmt', 'yuv420p', '-y', str(target)]
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errors)
            try:
                for frame in range(count):
                    seconds = min(recording.duration, frame/30.)
                    timestamp = recording.origin + round(seconds*1e9)
                    canvas = Image.new('RGB', (width, height))
                    draw = ImageDraw.Draw(canvas)
                    for column, role in enumerate(roles):
                        entries, times = recording.cameras[role]
                        if times[0] <= timestamp <= times[-1]:
                            pos = bisect_left(times, timestamp)
                            index = min((max(0, pos-1), min(len(times)-1, pos)), key=lambda i: abs(times[i]-timestamp))
                            item = entries[index]
                            rgb = Image.fromarray(recording.read_frame(role, item)).resize((320, 240))
                            raw = recording.read_frame(role, item, True)
                            scale = item.get('depth_scale_mm_per_unit', 1. if item.get('depth_unit') == 'mm' else None)
                            if scale is None: raise ValueError(role+' 缺少深度单位')
                            small = np.array(Image.fromarray(raw).resize((320, 240), resample=getattr(Image, 'Resampling', Image).NEAREST))
                            depth = Image.fromarray(depth_colors(small, float(scale), near, far))
                            canvas.paste(rgb, (column*320, 0)); canvas.paste(depth, (column*320, 240))
                        else:
                            draw.text((column*320+8, 110), 'No frame at this timestamp', fill='white')
                        for y, label in [(0, 'RGB'), (240, f'Depth {near:g}-{far:g}m')]:
                            draw.rectangle((column*320, y, column*320+319, y+20), fill='black')
                            draw.text((column*320+6, y+4), f'{role}  {label}', fill='white')
                    draw.rectangle((0, height-18, width, height), fill='black')
                    draw.text((8, height-15), f'{seconds:.2f} / {recording.duration:.2f} s', fill='white')
                    process.stdin.write(canvas.tobytes())
                    if frame % 30 == 0: progress(f'正在准备视频 {frame/count:.0%}（首次打开需等待，之后直接播放）')
                process.stdin.close()
                if process.wait(timeout=60):
                    errors.seek(0); raise RuntimeError(errors.read()[-1000:])
            except BaseException:
                if process.poll() is None: process.kill()
                process.wait()
                raise
        if key != source_signature(recording, near, far):
            raise ValueError('录制文件发生变化，未使用本次预览，请保存完成后重试。')
        target.replace(output)
    return {'path': str(output), 'filename': output.name, 'cached': False, 'duration': count/30., 'frames': count}


def prepare_video(args, path, update):
    if not args.ssh: return Path(render_video(path, progress=update)['path'])
    command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=3',
        '-o', 'ServerAliveCountMax=2', args.ssh, shlex.join(['nice', '-n', '10', 'taskset', '-c', '3-7',
        '/usr/bin/python3', '-u', args.remote_video_script, '--render', path])]
    result = None
    with tempfile.TemporaryFile(mode='w+') as errors:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors, text=True)
        for line in process.stdout:
            item = json.loads(line)
            if 'progress' in item: update(item['progress'])
            else: result = item
        if process.wait() or not result or not result.get('ok'):
            errors.seek(0); raise RuntimeError((result or {}).get('error') or errors.read()[-1200:] or '视频生成失败')
    name = result['filename']
    if (len(name) != 69 or not name.endswith('.webm') or any(c not in '0123456789abcdef' for c in name[:-5])):
        raise ValueError('视频缓存文件名无效')
    CACHE.mkdir(parents=True, exist_ok=True); local = CACHE/name
    if not local.exists():
        update('正在把视频预览传到本机…')
        with tempfile.TemporaryDirectory(prefix='download-', dir=CACHE) as tmp:
            target = Path(tmp)/name
            subprocess.run(['scp', '-q', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                args.ssh+':'+shlex.quote(result['path']), str(target)], check=True, timeout=180)
            target.replace(local)
    return local


def show_library(args):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk(); root.title('已录制 RGB + 深度视频'); root.geometry('900x380')
    body = ttk.Frame(root, padding=18); body.pack(fill='both', expand=True)
    ttk.Label(body, text='选择一条录制，像普通视频一样播放', font=('', 15)).pack(anchor='w', pady=(0, 12))
    choice = ttk.Combobox(body, state='readonly'); choice.pack(fill='x', pady=8)
    message = tk.StringVar(value='正在读取已录制数据…')
    ttk.Label(body, textvariable=message, wraplength=840).pack(anchor='w', pady=8)
    ttk.Label(body, text='视频上排：左腕 / 右腕 / 胸前 RGB；下排：对应深度。\n'
        '打开后可用播放器暂停、拖动进度和全屏。首次打开会准备视频，原始数据保持不变。',
        wraplength=840).pack(anchor='w', pady=12)
    pool = ThreadPoolExecutor(max_workers=1); messages = queue.Queue()
    state = {'future': None, 'kind': None, 'records': []}
    def submit(kind, fn):
        if state['future'] is not None: return
        state.update(kind=kind, future=pool.submit(fn)); play.state(['disabled']); refresh.state(['disabled'])
    def load():
        def job():
            transport = Transport(args)
            try: return transport.exchange({'action':'list','root':args.path or args.root})['recordings']
            finally: transport.close()
        submit('list', job)
    def start():
        i = choice.current()
        if i >= 0:
            path = state['records'][i]['path']; message.set('正在准备：'+path)
            submit('video', lambda: prepare_video(args, path, messages.put))
    row = ttk.Frame(body); row.pack(fill='x', pady=8)
    play = ttk.Button(row, text='▶ 播放所选视频', command=start); play.pack(side='left', padx=(0, 10))
    refresh = ttk.Button(row, text='刷新列表', command=load); refresh.pack(side='left')
    def raw():
        i=choice.current()
        if i >= 0:
            command=[sys.executable, str(Path(__file__).with_name('view_recorded_rgbd.py')), state['records'][i]['path']]
            if args.ssh: command += ['--ssh', args.ssh, '--remote-script', args.remote_script]
            subprocess.Popen(command)
    ttk.Button(row, text='逐帧检查…', command=raw).pack(side='left', padx=10)
    choice.bind('<Double-1>', lambda _event:start())
    def tick():
        while not messages.empty(): message.set(messages.get_nowait())
        f=state['future']
        if f is not None and f.done():
            kind=state['kind'];state['future']=None;play.state(['!disabled']);refresh.state(['!disabled'])
            try:
                result=f.result()
                if kind=='list':
                    state['records']=result;choice.configure(values=[r['label'] for r in result])
                    message.set(f'找到 {len(result)} 条录制。请选择后点击播放。')
                    if result:
                        choice.current(0)
                        if args.path:
                            for i,r in enumerate(result):
                                if r['path'] in (args.path, args.path.rstrip('/')+'/lerobot'):choice.current(i);break
                            start();args.path=None
                else:
                    subprocess.Popen(['totem', '--replace', str(result)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    message.set('视频已打开。可在播放器中暂停、拖动进度和全屏。\n'+str(result))
            except Exception as exc: message.set('打开失败：'+str(exc))
        root.after(100,tick)
    def close():
        if state['future'] is not None:
            message.set('正在准备视频，请完成后关闭窗口。');return
        pool.shutdown(wait=False);root.destroy()
    root.protocol('WM_DELETE_WINDOW',close);load();root.after(100,tick);root.mainloop()


def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('path',nargs='?');p.add_argument('--ssh');p.add_argument('--root',default='/home/nvidia/datasets/openarm_left_log_sessions')
    p.add_argument('--remote-script',default='/home/nvidia/dev/openarm-rgbd-preview/scripts/view_recorded_rgbd.py')
    p.add_argument('--remote-video-script',default='/home/nvidia/dev/openarm-rgbd-preview/scripts/play_recorded_rgbd.py')
    p.add_argument('--render',metavar='EPISODE');p.add_argument('--prepare',metavar='EPISODE',help='准备本机视频缓存并输出路径，不打开窗口')
    args=p.parse_args()
    if args.ssh and args.ssh.startswith('-'):p.error('SSH 主机名无效')
    if args.render:
        try:
            result=render_video(args.render,progress=lambda m:print(json.dumps({'progress':m},ensure_ascii=False),flush=True))
            print(json.dumps({'ok':True,**result}),flush=True)
        except Exception as exc:
            print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False),flush=True);raise SystemExit(1)
    elif args.prepare:print(prepare_video(args,args.prepare,lambda m:print(m,file=sys.stderr)))
    else:show_library(args)


if __name__=='__main__':main()
