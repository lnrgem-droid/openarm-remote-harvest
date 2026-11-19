#!/usr/bin/env python3
"""Terminate exact follower PIDs after power isolation or verified torque disable.

Internal helper for cleanup_rebooted_teleop and stop_without_gateway. Callers
must establish isolation or fresh all-motor disable proof; never run standalone.
Uses pidfds to avoid signaling a reused PID. No CAN, ROS, or process-group kill.
"""
import json
import os
from pathlib import Path
import select
import signal
import sys


def process_identity(pid):
    p = Path('/proc') / str(pid)
    try:
        return {'start_ticks': int((p / 'stat').read_text().rsplit(')', 1)[1].split()[19]),
                'argv': (p / 'cmdline').read_bytes().decode().split('\0')}
    except FileNotFoundError:
        return None


def terminate_exact(e):
    if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != e['boot_id']:
        raise RuntimeError('Jetson 启动身份已变化，未清理')
    expected = e['process_identities']
    if not expected or len(expected) > 4:
        raise RuntimeError('待清理进程集合无效')
    handles = {}
    try:
        # Open handles before the final identity check, closing the PID reuse race.
        for pid, value in expected.items():
            handles[pid] = os.pidfd_open(int(pid), 0)
            if process_identity(pid) != value:
                raise RuntimeError('机械臂进程已替换，未清理')
        # A launch process must not own unlisted children, since its graceful
        # shutdown can signal children. Refuse instead of touching other work.
        for p in Path('/proc').iterdir():
            if not p.name.isdigit() or p.name in expected:
                continue
            try:
                parent = p.joinpath('stat').read_text().rsplit(')', 1)[1].split()[1]
                if parent in expected:
                    raise RuntimeError('遥操进程有未核验的子进程，未清理')
            except FileNotFoundError:
                pass
        for pid, value in expected.items():
            if process_identity(pid) != value:
                raise RuntimeError('信号发送前进程身份已变化，未清理')
        for fd in handles.values():
            try:
                signal.pidfd_send_signal(fd, signal.SIGTERM)
            except ProcessLookupError:
                pass  # Exact process already exited; never signal its replacement.
        poll = select.poll()
        for fd in handles.values():
            poll.register(fd, select.POLLIN)
        import time
        pending = set(handles.values()); deadline = time.monotonic() + 16
        while pending and time.monotonic() < deadline:
            for fd, event in poll.poll(200):
                if event & select.POLLIN:
                    pending.discard(fd); poll.unregister(fd)
        if pending:
            raise RuntimeError('部分遥操进程尚未退出；未启动新控制器，可重新点击停止或清理重试')
        return {'stopped_pids': sorted(expected), 'started_controllers': False}
    finally:
        for fd in handles.values():
            os.close(fd)


if __name__ == '__main__':
    print(json.dumps(terminate_exact(json.load(sys.stdin))))
