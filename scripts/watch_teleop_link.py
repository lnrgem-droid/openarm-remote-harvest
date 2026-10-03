#!/usr/bin/env python3
"""Slow read-only operator diagnostics, NOT the real-time safety watchdog."""
import argparse
import copy
import fcntl
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import time
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from teleop_readiness import evaluate, motor_health, BOOT_PATH, session_id
from check_motor_enable import probe, combined

REMOTE = '''import json,socket,tempfile
with tempfile.TemporaryDirectory(prefix="oa_link_") as tmp:
 with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
  s.bind(tmp+"/reply");s.settimeout(.5)
  s.sendto(b'{"command":"status"}',"/tmp/openarm_remote_runtime.sock")
  print(s.recv(65536).decode())
'''


def describe(status, motors=None):
    result=evaluate(status, motors)
    return result['diagnostic'], result['message']


def remote_status(peer):
    r=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=2',peer,
                      '/usr/bin/python3 -'],input=REMOTE,text=True,capture_output=True,timeout=3,check=True)
    value=json.loads(r.stdout)
    if not isinstance(value,dict): raise ValueError('status must be an object')
    return value


class HealthMonitor:
    """Debounce diagnostic uncertainty; never clear evidence of drive loss.

    Counters belong to one leader session. Persisting them with the report
    prevents restarting this observer from bypassing an unresolved failure.
    This class only assesses samples; it never changes controller state.
    """
    REQUIRED_SAMPLES = 3

    def __init__(self, previous=None, boot_id=None):
        previous = previous if isinstance(previous, dict) else {}
        latch = previous.get('invalidated_session')
        self.latched = latch if type(latch) is int and latch > 0 else None
        self.evidence = copy.deepcopy(previous.get('invalidation_evidence'))
        if self.latched and (not isinstance(self.evidence, dict) or
                            self.evidence.get('leader_session_id') != self.latched):
            self.evidence = dict(leader_session_id=self.latched,
                diagnostic='HISTORY_UNKNOWN', observed_unix_s=None,
                message='历史会话已锁存，但旧版本未保留首次原因；必须明确重新初始化',
                status=None, motors=None)
        self.history = copy.deepcopy(previous.get('invalidation_history', []))
        if not isinstance(self.history, list): self.history = []
        same_boot = boot_id is None or previous.get('host_boot_id') == boot_id
        self.session = (previous.get('monitor_session_id') or session_id(previous.get('status'))) if same_boot else None
        if type(self.session) is not int or self.session <= 0: self.session = None
        failures = previous.get('consecutive_failures', 0) if same_boot else 0
        self.failures = failures if type(failures) is int and 0 <= failures <= 3 else 0
        # A monitor restart must collect a fresh complete recovery window.
        self.good = 0
        self.pending = True
        first = previous.get('uncertainty_evidence') if same_boot else None
        self.first_uncertain = copy.deepcopy(first) if isinstance(first, dict) else None

    def _capture(self, status, motors, assessment, now):
        return copy.deepcopy(dict(leader_session_id=self.session,
            diagnostic=assessment['diagnostic'], message=assessment['message'],
            observed_unix_s=now, status=status, motors=motors))

    def _latch(self, evidence, now):
        if not self.session or self.session == self.latched:
            return
        if any(isinstance(item, dict) and item.get('leader_session_id') == self.session
               for item in self.history):
            return
        if self.evidence:
            self.history.append(copy.deepcopy(self.evidence))
        self.latched = self.session
        self.evidence = copy.deepcopy(evidence)
        self.evidence['latched_unix_s'] = now

    def _session_latch(self):
        if any(isinstance(item, dict) and item.get('leader_session_id') == self.session
               for item in self.history):
            return self.session
        return self.latched

    def sample(self, status, motors, now=None):
        now = time.time() if now is None else now
        sid = session_id(status)
        if sid is not None and sid != self.session:
            self.session = sid
            self.failures = self.good = 0
            self.pending = True
            self.first_uncertain = None
        assessment = evaluate(status, motors, self._session_latch())
        code, message = motor_health(motors)
        if code in ('MOTOR_FAULT', 'MOTORS_DISABLED'):
            self._latch(self._capture(status, motors,
                dict(diagnostic=code, message=message), now), now)
        elif assessment['diagnostic'] == 'FAULT':
            # Explicit controller faults and E_STOP are not sampling jitter.
            # A later RUNNING reply in the same session cannot erase them.
            self._latch(self._capture(status, motors, assessment, now), now)
        if not assessment['teleop_ready']:
            self.failures = min(self.REQUIRED_SAMPLES, self.failures + 1)
            self.good = 0
            self.pending = True
            if self.first_uncertain is None:
                self.first_uncertain = self._capture(status, motors, assessment, now)
            if self.failures >= self.REQUIRED_SAMPLES:
                self._latch(self.first_uncertain, now)
        elif self.pending:
            self.failures = 0
            self.good += 1
            if self.good >= self.REQUIRED_SAMPLES:
                self.pending = False
                self.first_uncertain = None
        else:
            self.failures = 0
        # Apply a latch established by this sample before publishing readiness.
        assessment = evaluate(status, motors, self._session_latch())
        if assessment['teleop_ready'] and self.pending:
            assessment.update(teleop_ready=False, diagnostic='HEALTH_RECOVERING',
                message=f'健康采样恢复核验中：{self.good}/3 次；尚不能遥操',
                follow_enabled={'left':False, 'right':False})
        return dict(assessment, invalidated_session=self.latched,
            invalidation_evidence=copy.deepcopy(self.evidence),
            invalidation_history=copy.deepcopy(self.history),
            monitor_session_id=self.session, consecutive_failures=self.failures,
            recovery_pending=self.pending, recovery_good_samples=self.good,
            uncertainty_evidence=copy.deepcopy(self.first_uncertain))


def collect(peer):
    # One SSH round trip; overlap local and remote passive CAN acquisition.
    # Both status snapshots surround the remote probe to detect session swaps.
    source=Path(__file__).with_name('check_motor_enable.py').read_text()
    remote='namespace={"__name__":"passive_probe"}\nexec('+repr(source)+',namespace)\n'+'''
import json,socket,tempfile
def status():
 with tempfile.TemporaryDirectory(prefix="oa_health_") as tmp:
  with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
   s.bind(tmp+"/reply");s.settimeout(.5)
   s.sendto(b'{"command":"status"}',"/tmp/openarm_remote_runtime.sock")
   return json.loads(s.recv(65536))
before=status()
motors=namespace['probe'](['can1','can2'])
after=status()
print(json.dumps(dict(before=before,status=after,motors=motors)))
'''
    def read_remote():
        r=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=2',peer,
                          '/usr/bin/python3 -'],input=remote,text=True,capture_output=True,timeout=3,check=True)
        return json.loads(r.stdout)
    host = None
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            local=pool.submit(probe,['can0','can1'])
            remote_result=pool.submit(read_remote)
            host=local.result()
            peer_report=remote_result.result()
        status=peer_report['status']
        motors=combined(host,peer_report['motors'])
        if session_id(peer_report['before'])!=session_id(status):
            motors['error']='采样期间主臂会话改变，无法确认本次证据所属会话'
            motors['status_before']=peer_report['before']
            motors['status_after']=status
            return None,motors
    except (OSError,ValueError,KeyError,TypeError,AttributeError,subprocess.SubprocessError) as exc:
        # Preserve local drive evidence and the acquisition error even when
        # SSH/status fails. A total status loss still counts for the last session.
        return None, {'result':'UNKNOWN', 'host':host,
                      'follower':{'result':'UNKNOWN', 'error':str(exc)},
                      'error':type(exc).__name__+': '+str(exc)}
    return status,motors


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--jetson',default='openarm-jetson');p.add_argument('--output',required=True)
    p.add_argument('--once',action='store_true')
    p.add_argument('--owner-pid',type=int,help='exit when this control supervisor exits')
    a=p.parse_args()
    if a.jetson.startswith('-'): p.error('invalid SSH target')
    target=Path(a.output);target.parent.mkdir(parents=True,exist_ok=True)
    lock=target.with_suffix('.lock').open('a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('health monitor already running')
    boot=BOOT_PATH.read_text().strip()
    previous_report=None
    try:
        previous_report=json.loads(target.read_text())
    except (OSError,ValueError,AttributeError): pass
    monitor=HealthMonitor(previous_report, boot)
    def owner_identity():
        if a.owner_pid is None: return None
        return Path(f'/proc/{a.owner_pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    try: owner=owner_identity()
    except OSError: return
    previous=None
    while True:
        try:
            if owner_identity()!=owner: return
        except OSError: return
        started=time.monotonic_ns()
        status,motors=collect(a.jetson)
        assessment=monitor.sample(status,motors)
        key,message=assessment['diagnostic'],assessment['message']
        if previous!=key:
            print('运行状态检查：'+message+'。此检查不会重新使能、归位或恢复运动。',flush=True)
            previous=key
        report={**assessment,'schema_version':1,'host_boot_id':boot,
                'probe_started_monotonic_ns':started,'updated_unix_s':time.time(),
                'motion_recovery_performed':False,'status':status,'motors':motors}
        temporary=target.with_suffix('.pending')
        temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');temporary.replace(target)
        if a.once: return
        time.sleep(.5)


if __name__=='__main__': main()
