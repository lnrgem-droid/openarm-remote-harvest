#!/usr/bin/env python3
"""Explicit, one-shot physical saved-pose return acceptance, run on Jetson.

Requires cleared paths/empty grippers and an operator guarding the emergency
stop. Uses the same left_lock/right_return commands as the UI; never writes
saved poses, joint targets, encoder offsets, or bypasses controller gates.
"""
import argparse
import json
from pathlib import Path
import socket
import tempfile
import time


def request(command):
    with tempfile.TemporaryDirectory(prefix='oa_return_check_') as tmp:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.bind(tmp+'/reply'); sock.settimeout(1.)
            sock.sendto(json.dumps({'command': command}).encode(), '/tmp/openarm_remote_runtime.sock')
            result = json.loads(sock.recv(65536))
    if result.get('error'):
        raise RuntimeError(result['error'])
    return result


def healthy(s):
    return (s.get('state') == 'RUNNING' and s.get('fault_bits') == 0
            and s.get('feedback_fresh_for_control') is True
            and 0 <= s.get('action_age_ms', 999) < 100
            and 0 <= s.get('feedback_age_ms', 999) < 100)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true', help='Explicitly authorize one physical return')
    p.add_argument('--saved-unix-s', type=float, required=True, help='Expected saved-pose identity')
    p.add_argument('--output', required=True)
    args=p.parse_args()
    if not args.execute:
        p.error('Read description; --execute is required for physical motion')
    report={'passed':False, 'started_unix_s':time.time(), 'samples':[]}
    attempted=False
    try:
        start=request('status'); c=start.get('collection', {})
        if not healthy(start) or c.get('right_mode') != 'FOLLOW' or c.get('recording'):
            raise RuntimeError('Requires fresh RUNNING/right FOLLOW and no recording')
        if c.get('return_phase') != 'idle' or c.get('transitioning_arms'):
            raise RuntimeError('Motion transition is still active')
        if (c.get('saved') or {}).get('saved_unix_s') != args.saved_unix_s:
            raise RuntimeError('Saved pose identity changed; refusing motion')
        session=start['leader_session_id']
        locked=request('left_lock')
        left=locked['applied_axes'][:8]
        report['start']=start
        attempted=True
        request('right_return')
        t0=time.monotonic(); stable=None; last_print=-2.
        while time.monotonic()-t0 < 45.:
            s=request('status'); now=time.monotonic(); c=s.get('collection', {})
            report['samples'].append({'elapsed_s':now-t0, 'status':s})
            if now-t0-last_print >= 1.:
                print(json.dumps({'elapsed_s':round(now-t0,2), 'phase':c.get('return_phase'),
                    'right_mode':c.get('right_mode'), 'note':c.get('note'),
                    'detail':c.get('return_detail')}, ensure_ascii=False), flush=True)
                last_print=now-t0
            if not healthy(s) or s.get('leader_session_id') != session:
                raise RuntimeError('Control health/session changed; stopping acceptance')
            if c.get('saved') != start['collection']['saved']:
                raise RuntimeError('Saved pose changed during test')
            if c.get('left_mode') != 'HOLD' or s['applied_axes'][:8] != left:
                raise RuntimeError('Left HOLD target changed')
            if c.get('right_mode') == 'HOLD':
                raise RuntimeError(c.get('note', 'Return stopped in HOLD'))
            if c.get('right_mode') == 'FOLLOW' and c.get('return_phase') == 'idle' and not c.get('transitioning_arms'):
                target_error=max(abs(a-b) for a,b in zip(s['right_target_rad'], s['leader_right_rad']))
                if target_error > .01 or s['max_tracking_error_rad'] > .06:
                    raise RuntimeError('FOLLOW restored but target/correspondence not verified')
                stable=now if stable is None else stable
                if now-stable >= 5.:
                    report.update(passed=True, return_and_settle_s=stable-t0, final=s)
                    break
            else:
                stable=None
            time.sleep(.05)
        if not report['passed']:
            raise RuntimeError('Acceptance exceeded 45 s; no automatic retry')
    except Exception as exc:
        report['error']=str(exc)
        if attempted:
            try:
                report['paused']=request('right_pause')
            except Exception as stop_exc:
                report['pause_error']=str(stop_exc)
        print('NOT PASSED: '+str(exc), flush=True)
    finally:
        report['finished_unix_s']=time.time()
        Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','start','final','paused')}, ensure_ascii=False), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
