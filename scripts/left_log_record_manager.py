#!/usr/bin/env python3
"""Isolated left-log recorder using the proven batch/result persistence lifecycle."""
import argparse
import importlib.util
import json
import time
from pathlib import Path
import zmq
from left_log_common import ROLES,TASKS,ACTIVE,ERROR,CAMERA_STATUS,runtime_status,ready_reason

# A private module keeps the bimanual manager's task map unchanged, even in tests.
spec=importlib.util.spec_from_file_location('_left_log_lifecycle',Path(__file__).with_name('jetson_record_manager.py'))
base=importlib.util.module_from_spec(spec); spec.loader.exec_module(base)
base.TASK_GROUPS=dict(TASKS)
base.GROUP_SIDES={group:'left' for group in TASKS.values()}
base.EPISODE_GROUPS=tuple(TASKS.values())


class LeftLogRecorder(base.Recorder):
    camera_roles=ROLES
    session_prefix='left_log'

    def __init__(self,root,*,runtime_dir=None,storage_root=None):
        super().__init__(root,runtime_dir=runtime_dir or Path.home()/'openarm-left-log-runtime',
                         storage_root=storage_root or Path.home()/'datasets/openarm_left_log_sessions')
        self.active_marker=ACTIVE; self.error_marker=ERROR; self.camera_status_path=CAMERA_STATUS
        if self.session_root is None: self.session_base=self.allowed_storage_root
        self.recording_leader_session=None

    def _motion_request(self,command,**fields):
        if command=='status': return runtime_status()
        if command=='collection_end': return {}  # release only this recorder's local episode ownership
        raise RuntimeError('录制器只读取左臂状态，不发送运动指令')

    def _acquire_motion_lock(self,task_group,token):
        s=runtime_status()
        reason=ready_reason(s)
        if reason: raise RuntimeError(reason)
        self.recording_leader_session=s['leader_session_id']
        return s

    def _release_motion_lock(self):
        self.motion_token=None

    def _recording_motion_valid(self,motion):
        return not ready_reason(motion) and motion['leader_session_id']==self.recording_leader_session

    def _recorder_command(self):
        return ['bash',str(self.root/'scripts/record_left_log_dataset.sh')]

    def _write_session_manifest(self):
        super()._write_session_manifest()
        if self.session_root is None: return
        path=self.session_root/'session.json'; value=json.loads(path.read_text())
        value.update(profile='left_log_v1',active_arms=['left'],vector_size=8,
                     episode_layout={group:f'episodes/{group}/episode_NNNN' for group in TASKS.values()},
                     storage_contract='30 Hz real left J1..J7 + gripper state/action; chest and left wrist lossless RGB-D.')
        self._write_json(path,value)

    def _load_session(self,root,**kwargs):
        value=json.loads((Path(root)/'session.json').read_text())
        if value.get('profile')!='left_log_v1': raise ValueError('只能继续单左臂菌棒批次，不能混用双臂数据')
        super()._load_session(root,**kwargs)

    def _refresh_numbers_from_disk(self):
        # On-disk directories are authoritative after manual deletion. Incomplete
        # directories also reserve their number. Never change an active episode.
        if (self.session_root is None or self.active_episode is not None
                or (self.process is not None and self.process.poll() is None)
                or getattr(self, '_finalizing', False)
                or self.phase in {'starting', 'recording', 'stopping', 'awaiting_result'}):
            return
        for group in TASKS.values():
            highest = 0
            for path in (self.session_root / 'episodes' / group).glob('episode_*'):
                try:
                    highest = max(highest, int(path.name.removeprefix('episode_')))
                except ValueError:
                    continue
            self.next_episode_by_task[group] = highest + 1
        if self.last_episode and not (Path(self.last_episode['episode_root']) / 'episode.json').is_file():
            self.last_episode = None

    @base.synchronized
    def prepare_storage(self):
        current = self.status()
        if current['running']:
            # Includes recovered old-batch data still awaiting an operator label.
            return {'ok': True, **current}
        fixed = self._safe_storage_path(self.allowed_storage_root, create=True)
        if self.session_root != fixed:
            if (fixed / 'session.json').exists():
                self._load_session(fixed)
            else:
                # Recover the layout even if only session.json was removed.
                # _load_session scans pending metadata and reserves existing numbers.
                self._write_json(fixed / 'session.json', {
                    'profile': 'left_log_v1', 'session_id': 'left_log_fixed',
                    'session_started_unix_s': time.time(), 'session_ended_unix_s': None,
                })
                (fixed / 'episodes').mkdir(exist_ok=True)
                self._load_session(fixed)
        for group in TASKS.values():
            (fixed / 'episodes' / group).mkdir(parents=True, exist_ok=True)
        self._refresh_numbers_from_disk()
        self._write_session_manifest()
        self._persist_session()
        return {'ok': True, **self.status()}

    @base.synchronized
    def start_session(self, mode='continue', storage_base='', session_root=''):
        # Old UI requests cannot accidentally create another dated batch.
        if storage_base or session_root or mode not in {'new', 'continue'}:
            return {'ok': False, 'error': '当前使用固定目录，无需新建或选择批次', **self.status()}
        return self.prepare_storage()

    @base.synchronized
    def start_episode(self,task,target=''):
        if task not in TASKS: return {'ok':False,'error':'仅支持左臂拉出/推回菌棒',**self.status()}
        self.prepare_storage()
        return super().start_episode(task,target)

    @base.synchronized
    def status(self):
        value=super().status()
        self._refresh_numbers_from_disk()
        fixed = self.allowed_storage_root.resolve()
        value.update(profile='left_log_v1',active_arms=['left'],vector_size=8,camera_roles=list(ROLES),
                     storage_mode='fixed', storage_ready=self.session_root == fixed,
                     next_episode_by_task=dict(self.next_episode_by_task), last_episode=self.last_episode,
                     task_directories={task: str(fixed / 'episodes' / group) for task, group in TASKS.items()})
        return value


def main():
    p=argparse.ArgumentParser(); p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--bind',default='tcp://*:5597'); a=p.parse_args()
    recorder=LeftLogRecorder(a.root)
    recorder.prepare_storage()
    context=zmq.Context(); sock=context.socket(zmq.REP); sock.setsockopt(zmq.LINGER,0); sock.bind(a.bind)
    try:
        while True:
            raw=sock.recv()
            try:
                request=json.loads(raw)
                if not isinstance(request,dict) or request.get('command')=='start':
                    raise ValueError('请选择菌棒任务后开始录制')
                if recorder.session_root != recorder.allowed_storage_root.resolve():
                    recorder.prepare_storage()
                response=base.dispatch_request(recorder,request)
            except Exception as exc: response={'ok':False,'error':str(exc),**recorder.status()}
            sock.send_json(response)
    finally: sock.close(0); context.term()

if __name__=='__main__': main()
