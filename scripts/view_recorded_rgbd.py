#!/usr/bin/env python3
"""Read-only RGB-D recording player, local or over SSH.

  /usr/bin/python3 view_recorded_rgbd.py /path/to/episode_or_lerobot
  /usr/bin/python3 view_recorded_rgbd.py --ssh openarm-jetson --root /home/nvidia/datasets

Requires numpy, Pillow and (for the GUI only) tkinter. No ROS/CAN access.
"""
from __future__ import annotations

import argparse
import base64
from bisect import bisect_left, bisect_right
from concurrent.futures import ThreadPoolExecutor
import io
import json
import math
from pathlib import Path
import os
import selectors
import shlex
import subprocess
import sys
import time

import numpy as np
from PIL import Image

ROLE_LABELS = {'left_wrist': '左腕', 'right_wrist': '右腕', 'chest': '胸前'}
TASK_LABELS = {'LEFT_PICK_ONE': '左臂采摘', 'RIGHT_PULL_LOG': '拉出菌棒',
               'RIGHT_PUSH_LOG': '推回菌棒', 'LEFT_GRASP_LOG': '左臂抓取菌棒'}


def neighbor_time(timeline, seconds, direction):
    """Step on the union of camera timestamps, including unsynchronized frames."""
    if not timeline:
        return seconds
    index = (bisect_right(timeline, seconds+1e-9) if direction > 0
             else bisect_left(timeline, seconds-1e-9)-1)
    return timeline[max(0, min(len(timeline)-1, index))]


def read_json(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def discover(root):
    root = Path(root).expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f'数据目录不存在：{root}')
    folders = [root] if (root/'depth_raw').is_dir() else []
    folders += [p.parent for p in root.rglob('depth_raw') if p.is_dir()]
    result = []
    for folder in sorted(set(folders), key=lambda p: p.stat().st_mtime, reverse=True):
        if not list((folder/'depth_raw').glob('*.jsonl')):
            continue
        info = read_json(folder/'meta/info.json')
        episode = read_json(folder.parent/'episode.json')
        task = episode.get('task') or info.get('task') or '未知任务'
        result.append({'path': str(folder), 'task': task,
                       'result': episode.get('result', '未标记'),
                       'complete': (folder/'rgbd-complete.json').is_file(),
                       'label': f"{TASK_LABELS.get(task, task)} · {episode.get('result', '未标记')} · {folder.relative_to(root) if folder != root else folder.name}"})
    return result


def depth_colors(raw, scale_mm, near_m, far_m):
    if not (math.isfinite(scale_mm) and scale_mm > 0
            and math.isfinite(near_m) and math.isfinite(far_m) and 0 <= near_m < far_m):
        raise ValueError('深度范围必须满足 0 ≤ 最近距离 < 最远距离，单位为米')
    meters = raw.astype(np.float32) * (scale_mm / 1000.)
    fraction = np.clip((meters-near_m)/(far_m-near_m), 0., 1.)
    # A fixed scale for the entire clip: warm = near, cool = far; zero = invalid.
    rgb = np.stack((1.-fraction, 1.-np.abs(2.*fraction-1.), fraction), axis=-1)
    rgb = np.uint8(rgb*255)
    rgb[raw == 0] = 0
    return rgb


class Recording:
    def __init__(self, path):
        root = Path(path).expanduser().resolve()
        if not (root/'depth_raw').is_dir() and (root/'lerobot/depth_raw').is_dir():
            root = root/'lerobot'
        self.root = root
        self.cameras = {}
        self.warnings = []
        for index in sorted((root/'depth_raw').glob('*.jsonl')):
            role = index.stem
            if not (root/'rgb_raw'/f'{role}.rgb24').is_file() or not (root/'depth_raw'/f'{role}.u16le').is_file():
                self.warnings.append(f'{role} 缺少 RGB 或深度原始文件')
                continue
            entries = []
            with index.open(encoding='utf-8') as stream:
                for number, line in enumerate(stream, 1):
                    try:
                        item = json.loads(line)
                        if item.get('role') != role or item.get('aligned_depth_to_rgb') is not True:
                            continue
                        if int(item['host_realtime_ns']) <= 0:
                            raise ValueError('时间戳无效')
                        entries.append(item)
                    except (ValueError, KeyError, TypeError) as exc:
                        self.warnings.append(f'{role} 索引第 {number} 行无效：{exc}')
            entries.sort(key=lambda item: int(item['host_realtime_ns']))
            if entries:
                self.cameras[role] = (entries, [int(item['host_realtime_ns']) for item in entries])
        if not self.cameras:
            raise ValueError('没有可回放的原始 RGB-D 数据，请选择包含 depth_raw/rgb_raw 的目录或它的 episode 上级目录')
        self.origin = min(times[0] for _, times in self.cameras.values())
        self.end = max(times[-1] for _, times in self.cameras.values())
        self.duration = (self.end-self.origin)/1e9
        self.timeline = [(stamp-self.origin)/1e9 for stamp in sorted(
            {stamp for _, times in self.cameras.values() for stamp in times})]

    def metadata(self):
        return {'path': str(self.root), 'duration': self.duration, 'timeline': self.timeline,
                'roles': list(self.cameras),
                'frames': {role: len(entries) for role, (entries, _) in self.cameras.items()},
                'complete': (self.root/'rgbd-complete.json').is_file(),
                'warnings': self.warnings[:6]}

    def read_frame(self, role, item, depth=False):
        h, w = int(item['height']), int(item['width'])
        if not (0 < h <= 8192 and 0 < w <= 8192):
            raise ValueError('图像尺寸无效')
        offset = int(item['offset_bytes' if depth else 'rgb_offset_bytes'])
        count = int(item['nbytes' if depth else 'rgb_nbytes'])
        expected = h*w*(2 if depth else 3)
        if offset < 0 or count != expected:
            raise ValueError(f'{role} 帧大小或偏移无效')
        path = self.root/('depth_raw' if depth else 'rgb_raw')/(role+('.u16le' if depth else '.rgb24'))
        with path.open('rb') as f:
            f.seek(offset)
            data = f.read(count)
        if len(data) != expected:
            raise ValueError(f'{role} 数据不完整：需要 {expected} 字节，实际 {len(data)} 字节')
        return np.frombuffer(data, dtype='<u2' if depth else 'u1').reshape((h, w) if depth else (h, w, 3))

    def frame(self, seconds, near=0.1, far=3.):
        if not math.isfinite(seconds):
            raise ValueError('播放时间无效')
        target = self.origin+round(max(0., min(self.duration, seconds))*1e9)
        images = {}
        for role, (entries, times) in self.cameras.items():
            if target < times[0] or target > times[-1]:
                images[role] = {'unavailable': '该相机在此时刻没有画面'}
                continue
            pos = bisect_left(times, target)
            index = min((max(0, pos-1), min(len(times)-1, pos)), key=lambda i: abs(times[i]-target))
            item = entries[index]
            depth = self.read_frame(role, item, True)
            scale = item.get('depth_scale_mm_per_unit')
            if scale is None:
                if item.get('depth_unit') != 'mm':
                    raise ValueError(f'{role} 缺少深度单位，无法正确显示距离')
                scale = 1.
            colored = depth_colors(depth, float(scale), near, far)
            encoded = []
            for array in (self.read_frame(role, item), colored):
                picture = Image.fromarray(array)
                picture.thumbnail((640, 480))
                buf = io.BytesIO(); picture.save(buf, format='JPEG', quality=82)
                encoded.append(base64.b64encode(buf.getvalue()).decode('ascii'))
            images[role] = {'rgb': encoded[0], 'depth': encoded[1], 'frame': index+1,
                            'time': (times[index]-self.origin)/1e9,
                            'skew_ms': (times[index]-target)/1e6}
        return {'time': (target-self.origin)/1e9, 'images': images}


def worker():
    recording = None
    for line in sys.stdin:
        try:
            request = json.loads(line)
            action = request['action']
            if action == 'list': result = {'recordings': discover(request['root'])}
            elif action == 'open':
                recording = Recording(request['path']); result = recording.metadata()
            elif action == 'frame' and recording is not None:
                result = recording.frame(float(request['time']), float(request.get('near', .1)), float(request.get('far', 3.)))
            else: raise ValueError('请先打开录制数据')
            print(json.dumps({'ok': True, **result}, ensure_ascii=False), flush=True)
        except Exception as exc:
            print(json.dumps({'ok': False, 'error': str(exc)}, ensure_ascii=False), flush=True)


class Transport:
    def __init__(self, args):
        if args.ssh:
            if args.ssh.startswith('-'): raise ValueError('SSH 主机名无效')
            command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                       '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2', args.ssh,
                       shlex.join(['/usr/bin/python3', '-u', args.remote_script, '--worker'])]
        else:
            command = [sys.executable, '-u', str(Path(__file__).resolve()), '--worker']
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.buffer = b''

    def exchange(self, request):
        if self.process.poll() is not None: raise RuntimeError('数据读取进程已退出，请检查 SSH 连接后重新打开工具')
        self.process.stdin.write((json.dumps(request)+'\n').encode()); self.process.stdin.flush()
        deadline = time.monotonic()+20
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            while b'\n' not in self.buffer:
                left = deadline-time.monotonic()
                if left <= 0 or not selector.select(left):
                    self.close()
                    raise TimeoutError('读取数据超时，请检查 Jetson 连接后重新打开工具')
                data = os.read(self.process.stdout.fileno(), 65536)
                if not data: raise RuntimeError('数据读取连接中断，请检查终端中的错误信息')
                self.buffer += data
                if len(self.buffer) > 16*1024*1024: raise ValueError('回放响应过大')
        line, self.buffer = self.buffer.split(b'\n', 1)
        result = json.loads(line)
        if not result.get('ok'): raise ValueError(result.get('error', '读取失败'))
        return result

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try: self.process.wait(timeout=1)
            except subprocess.TimeoutExpired: self.process.kill()


def show_player(args):
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    from PIL import ImageTk

    root = tk.Tk(); root.title('OpenArm 录制数据回放｜RGB + 深度')
    root.tk.call('tk', 'scaling', 1.0)
    root.geometry('1400x900'); root.minsize(1000, 650)
    transport = Transport(args)
    pool = ThreadPoolExecutor(max_workers=1)
    state = {'future': None, 'kind': None, 'playing': False, 'duration': 0.,
             'time': 0., 'drawn': None, 'loaded': False, 'roles': [], 'records': [], 'closed': False}
    photos, panels = {}, {}
    info = tk.StringVar(value='正在读取数据目录…')
    location = tk.StringVar(value=str(args.root))
    choice = tk.StringVar(); speed = tk.StringVar(value='1.0')
    mode = tk.StringVar(value='按时间播放')
    loop = tk.BooleanVar(value=False)
    near = tk.StringVar(value='0.1'); far = tk.StringVar(value='3.0')
    depth_range = [.1, 3.]
    top = ttk.Frame(root, padding=10); top.pack(fill='x')
    ttk.Label(top, text=('Jetson 数据目录' if args.ssh else '本机数据目录')).grid(row=0, column=0, sticky='w')
    ttk.Entry(top, textvariable=location).grid(row=0, column=1, sticky='ew', padx=8)
    top.columnconfigure(1, weight=1)
    selector = ttk.Combobox(top, textvariable=choice, state='readonly'); selector.grid(row=1, column=0, columnspan=3, sticky='ew', pady=8)
    ttk.Label(top, textvariable=info, wraplength=1300).grid(row=2, column=0, columnspan=4, sticky='w')
    controls = ttk.Frame(root, padding=10); controls.pack(side='bottom', fill='x')
    inspection = ttk.Frame(root, padding=(10, 0)); inspection.pack(side='bottom', fill='x')
    frame_info = tk.StringVar(value='')
    ttk.Label(inspection, textvariable=frame_info).pack(side='right')
    progress = tk.DoubleVar(value=0.)
    clock = ttk.Label(controls, text='0.00 / 0.00 s'); clock.pack(side='right')
    track = ttk.Scale(controls, from_=0, to=1, variable=progress); track.pack(side='bottom', fill='x', pady=8)
    canvas = ttk.Frame(root, padding=8); canvas.pack(fill='both', expand=True)

    def submit(kind, request):
        if state['future'] is not None: return False
        state['kind'] = kind; state['future'] = pool.submit(transport.exchange, request)
        return True

    def refresh():
        if state['future'] is not None: return
        state['playing'] = False
        info.set('正在读取目录…')
        submit('list', {'action': 'list', 'root': location.get()})

    def open_path(path):
        if state['future'] is not None:
            state['open_next'] = path; state['playing'] = False
            return
        state.update(playing=False, loaded=False)
        info.set('正在打开：'+path)
        submit('open', {'action': 'open', 'path': path})

    def selected(_event=None):
        index = selector.current()
        if index >= 0: open_path(state['records'][index]['path'])

    def pick_local():
        path = filedialog.askdirectory(title='选择 episode 或包含 depth_raw/rgb_raw 的数据目录')
        if path: open_path(path)

    def seek(value):
        state['time'] = float(value); state['anchor'] = time.monotonic(); state['base'] = state['time']
        state['drawn'] = None
        state['next_due'] = time.monotonic()

    def step(direction):
        if not state['loaded']: return
        state['playing'] = False
        seek(neighbor_time(state['timeline'], state['time'], direction))

    def toggle():
        if not state['loaded']: return
        if state['time'] >= state['duration']: seek(0.)
        state['playing'] = not state['playing']
        state['anchor'], state['base'] = time.monotonic(), state['time']
        state['next_due'] = time.monotonic()

    def change_speed(_event=None):
        state['anchor'], state['base'] = time.monotonic(), state['time']
        state['next_due'] = time.monotonic()

    def apply_depth():
        try:
            a, b = float(near.get()), float(far.get())
            depth_colors(np.zeros((1, 1), dtype=np.uint16), 1., a, b)
            depth_range[:] = [a, b]; state['drawn'] = None
        except ValueError as exc: messagebox.showerror('深度范围无效', str(exc))

    ttk.Button(top, text='刷新列表', command=refresh).grid(row=0, column=2, padx=8)
    if not args.ssh: ttk.Button(top, text='打开本地文件夹…', command=pick_local).grid(row=0, column=3)
    ttk.Button(top, text='打开所选条目', command=selected).grid(row=1, column=3, padx=8)
    selector.bind('<<ComboboxSelected>>', selected)
    play = ttk.Button(controls, text='播放', command=toggle); play.pack(side='left', padx=4)
    ttk.Button(controls, text='回到开头', command=lambda: seek(0.)).pack(side='left', padx=4)
    ttk.Button(inspection, text='◀ 上一帧', command=lambda: step(-1)).pack(side='left', padx=4)
    ttk.Button(inspection, text='下一帧 ▶', command=lambda: step(1)).pack(side='left', padx=4)
    playback_mode = ttk.Combobox(inspection, textvariable=mode,
        values=['按时间播放', '逐帧不跳帧'], state='readonly', width=13)
    playback_mode.pack(side='left', padx=8); playback_mode.bind('<<ComboboxSelected>>', change_speed)
    ttk.Checkbutton(inspection, text='循环播放', variable=loop).pack(side='left')
    ttk.Label(controls, text='速度').pack(side='left', padx=4)
    rate = ttk.Combobox(controls, textvariable=speed, values=['0.25', '0.5', '1.0', '2.0'], width=5, state='readonly')
    rate.pack(side='left'); rate.bind('<<ComboboxSelected>>', change_speed)
    ttk.Label(controls, text='深度范围（米）').pack(side='left', padx=(18, 4))
    ttk.Entry(controls, textvariable=near, width=5).pack(side='left')
    ttk.Label(controls, text='至').pack(side='left')
    ttk.Entry(controls, textvariable=far, width=5).pack(side='left')
    ttk.Button(controls, text='应用', command=apply_depth).pack(side='left', padx=4)
    ttk.Label(controls, text='暖色近 · 冷色远 · 黑色无效').pack(side='left', padx=8)
    track.configure(command=seek)
    root.bind('<space>', lambda event: toggle() if event.widget.winfo_class() not in {'TEntry', 'TCombobox'} else None)
    for key, direction in [('<Left>', -1), ('<Right>', 1)]:
        root.bind(key, lambda event, d=direction: step(d)
                  if event.widget.winfo_class() not in {'TEntry', 'TCombobox', 'TScale'} else None)

    def build_panels(roles):
        for child in canvas.winfo_children(): child.destroy()
        panels.clear(); photos.clear()
        for column, role in enumerate(roles):
            canvas.columnconfigure(column, weight=1, uniform='cameras')
            for row, mode in enumerate(['rgb', 'depth']):
                box = ttk.LabelFrame(canvas, text=f"{ROLE_LABELS.get(role, role)} · {'彩色 RGB' if mode == 'rgb' else '深度（伪彩色）'}")
                box.grid(row=row, column=column, sticky='nsew', padx=4, pady=4)
                label = ttk.Label(box, text='等待画面', anchor='center'); label.pack(fill='both', expand=True)
                panels[role, mode] = label
        canvas.rowconfigure(0, weight=1, uniform='rows'); canvas.rowconfigure(1, weight=1, uniform='rows')

    def render(result):
        for role, item in result['images'].items():
            for mode in ['rgb', 'depth']:
                label = panels[role, mode]
                if 'unavailable' in item:
                    label.configure(image='', text=item['unavailable']); continue
                image = Image.open(io.BytesIO(base64.b64decode(item[mode])))
                image.thumbnail((max(100, canvas.winfo_width()//len(state['roles'])-24), max(80, canvas.winfo_height()//2-38)))
                photos[role, mode] = ImageTk.PhotoImage(image)
                label.configure(image=photos[role, mode], text='')
        frame_info.set(' ｜ '.join(f"{ROLE_LABELS.get(role,role)} {item['frame']}/{state['frames'][role]}帧"
            for role, item in result['images'].items() if 'frame' in item))
        gaps = [ROLE_LABELS.get(role, role) for role, item in result['images'].items() if abs(item.get('skew_ms', 0)) > 150]
        info.set(state['description'] + ('；此处相机帧存在时间缺口：'+','.join(gaps) if gaps else ''))

    def tick():
        if state['closed']: return
        future = state['future']
        if future is not None and future.done():
            kind = state['kind']; state['future'] = None
            try:
                result = future.result()
                if kind == 'list':
                    state['records'] = result['recordings']
                    selector.configure(values=[r['label'] for r in state['records']])
                    info.set(f"找到 {len(state['records'])} 条录制，请选择条目。" if state['records'] else '此目录下没有找到原始 RGB-D 录制。')
                    if state['records']:
                        selector.current(0); selected()
                elif kind == 'open':
                    state.update(duration=result['duration'], loaded=True, roles=result['roles'], time=0., drawn=None,
                                 timeline=result['timeline'], frames=result['frames'], next_due=time.monotonic())
                    state['description'] = result['path'] + ' ｜ ' + '，'.join(f"{ROLE_LABELS.get(r,r)} {n}帧" for r,n in result['frames'].items())
                    if not result['complete']: state['description'] += ' ｜ 未发现封口标记，仅查看当前已有帧'
                    if result['warnings']: state['description'] += ' ｜ '+ '；'.join(result['warnings'])
                    track.configure(to=max(.001, result['duration'])); progress.set(0.)
                    info.set(state['description']); build_panels(result['roles'])
                    if args.autoplay: toggle()
                elif kind == 'frame':
                    render(result)
                    next_time = neighbor_time(state['timeline'], result['time'], 1)
                    state['next_due'] = time.monotonic() + max(0., next_time-result['time'])/float(speed.get())
            except Exception as exc:
                state['playing'] = False; info.set('读取失败：'+str(exc))
                if kind == 'open': state['loaded'] = False
        if state.get('open_next') and state['future'] is None:
            open_path(state.pop('open_next'))
        if state['loaded']:
            if state['playing'] and mode.get() == '按时间播放':
                state['time'] = min(state['duration'], state['base']+(time.monotonic()-state['anchor'])*float(speed.get()))
                if state['time'] >= state['duration']:
                    if loop.get(): seek(0.)
                    else: state['playing'] = False
            elif (state['playing'] and state['future'] is None and state['drawn'] == state['time']
                  and time.monotonic() >= state['next_due']):
                next_time = neighbor_time(state['timeline'], state['time'], 1)
                if next_time > state['time']: state['time'] = next_time
                elif loop.get(): seek(0.)
                else: state['playing'] = False
            progress.set(state['time']); clock.configure(text=f"{state['time']:.2f} / {state['duration']:.2f} s")
            if state['future'] is None and state['drawn'] != state['time']:
                state['drawn'] = state['time']
                submit('frame', {'action': 'frame', 'time': state['time'], 'near': depth_range[0], 'far': depth_range[1]})
        play.configure(text='暂停' if state['playing'] else '播放')
        root.after(15, tick)

    def close():
        state['closed'] = True; transport.close(); pool.shutdown(wait=False, cancel_futures=True); root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)
    if args.path: open_path(args.path)
    else: refresh()
    root.after(66, tick); root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('path', nargs='?', help='episode 或包含 depth_raw/rgb_raw 的目录')
    parser.add_argument('--ssh', help='通过 SSH 读取 Jetson，例如 openarm-jetson')
    parser.add_argument('--root', default='/home/nvidia/datasets', help='录制数据搜索目录')
    parser.add_argument('--remote-script', default='/home/nvidia/dev/openarm-rgbd-preview/scripts/view_recorded_rgbd.py')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--list', action='store_true', help='列出录制数据，不打开界面')
    parser.add_argument('--autoplay', action='store_true', help='打开录制后自动播放')
    args = parser.parse_args()
    if args.worker: worker()
    elif args.list:
        transport = Transport(args)
        try: print(json.dumps(transport.exchange({'action': 'list', 'root': args.root}), ensure_ascii=False, indent=2))
        finally: transport.close()
    else: show_player(args)


if __name__ == '__main__':
    main()
