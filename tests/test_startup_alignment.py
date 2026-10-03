"""Synthetic pose traces; no ROS, CAN transmission, or robot motion."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('alignment', ROOT / 'scripts/verify_startup_alignment.py')
alignment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(alignment)
POLICY = json.loads((ROOT / 'scripts/startup_alignment_policy.json').read_text())


def status(phase='aligning'):
    s = {'state': 'ALIGNING' if phase == 'aligning' else 'RUNNING', 'fault_bits': 0,
         'feedback_fresh_for_control': True, 'action_age_ms': 1., 'feedback_age_ms': 2.,
         'leader_session_id': 123, 'enabled_arms': ['left', 'right'],
         'relative_follow_reference_captured': True,
         'collection': {'left_mode': 'FOLLOW', 'right_mode': 'FOLLOW',
                        'recording': None, 'transitioning_arms': [], 'return_phase': 'idle'}}
    for side in ('left', 'right'):
        for key in ('leader_' + side + '_rad', side + '_actual_rad', side + '_target_rad'):
            s[key] = list(POLICY['home_rad'])
    return s


class Clock:
    now = 0.

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def run_trace(read, phase='aligning'):
    clock = Clock()
    return alignment.verify(lambda: read(clock.now), phase, POLICY, clock.time, clock.sleep)


def test_policy_validates_and_prevents_loosening_past_home_limit():
    alignment.validate_policy(POLICY)
    for key, value in (('pair_tolerance_rad', [.15]*7), ('home_tolerance_rad', .15),
                       ('stable_duration_s', 0), ('timeout_s', float('nan')),
                       ('target_tolerance_rad', float('inf')), ('home_rad', [0]*6)):
        p = copy.deepcopy(POLICY); p[key] = value
        with pytest.raises(ValueError):
            alignment.validate_policy(p)


@pytest.mark.parametrize('phase', ['aligning', 'following'])
def test_good_trace_requires_stable_window(phase):
    result = run_trace(lambda t: status(phase), phase)
    assert result['accepted'] and result['samples'] >= 11
    assert len(result['joints']) == 14


def test_seven_degree_j7_mismatch_never_passes():
    s = status(); s['left_actual_rad'][6] = .12
    result = run_trace(lambda t: s)
    assert not result['accepted']
    assert any('左 J7' in issue for issue in result['issues'])


def test_identical_but_wrong_home_is_not_alignment_success():
    s = status()
    s['left_actual_rad'][3] += .10
    s['leader_left_rad'][3] += .10
    result = run_trace(lambda t: s)
    assert not result['accepted']
    assert any('未到初始位' in issue for issue in result['issues'])


def test_normal_gain_droop_after_startup_hold_is_rejected():
    s = status('following'); s['left_actual_rad'][6] -= .047
    assert not run_trace(lambda t: s, 'following')['accepted']


def test_clipped_or_offset_target_cannot_hide_mismatch():
    s = status('following'); s['right_target_rad'][6] = .02
    result = run_trace(lambda t: s, 'following')
    assert not result['accepted']
    assert any('下发目标未与主臂一一对应' in issue for issue in result['issues'])


@pytest.mark.parametrize('field,value', [
    ('action_age_ms', 101.), ('feedback_age_ms', float('nan')),
    ('feedback_fresh_for_control', False), ('enabled_arms', ['left']),
    ('leader_left_rad', [0]*6), ('left_actual_rad', [float('nan')]*7),
    ('leader_session_id', None), ('fault_bits', 8), ('state', 'E_STOP')])
def test_invalid_or_unsafe_feedback_rejected(field, value):
    s = status(); s[field] = value
    assert not run_trace(lambda t: s)['accepted']


def test_moving_together_does_not_pass_static_gate():
    def moving(t):
        s = status()
        delta = .018 if int(t * 10) % 2 else 0.
        s['leader_left_rad'][0] += delta
        s['left_actual_rad'][0] += delta
        return s
    result = run_trace(moving)
    assert not result['accepted']
    assert '静止' in result['issues'][0]


def test_bad_sample_resets_stable_window_then_can_settle():
    def settling(t):
        s = status()
        if t < 2.:
            s['left_actual_rad'][6] = .12
        return s
    result = run_trace(settling)
    assert result['accepted'] and result['samples'] >= 30


def test_session_change_aborts_instead_of_joining_old_and_new_samples():
    def restart(t):
        s = status(); s['leader_session_id'] = 456 if t > .5 else 123
        return s
    assert not run_trace(restart)['accepted']


def test_transport_gap_resets_window():
    window = alignment.StableWindow(POLICY)
    result = alignment.assess(status(), 'aligning', POLICY)
    for t in (0., .1, .2, .3, 1.0, 1.1, 1.2):
        assert not window.add(t, result)


def test_only_matching_successful_startup_report_allows_reuse(tmp_path):
    s = status('following')
    report_path = tmp_path / 'report.json'
    check = str(ROOT / 'scripts/check_remote_running_status.py')
    for report, good in (({}, False), ({'accepted': False}, False),
                         ({'accepted': True, 'phase': 'following', 'leader_session_id': 456}, False),
                         ({'accepted': True, 'phase': 'following', 'leader_session_id': 123}, True)):
        report_path.write_text(json.dumps(report))
        result = subprocess.run(['/usr/bin/python3', check, '--startup-report', str(report_path)],
                                input=json.dumps(s), text=True, capture_output=True)
        assert (result.returncode == 0) == good


def test_launcher_checks_success_and_not_just_end_of_homing():
    text = (ROOT / 'scripts/run_bimanual_remote_feedback.sh').read_text()
    assert "grep -c 'Startup homing command complete.'" not in text
    assert text.count('remote_control align ') == 1
    assert text.index('verify_alignment aligning ||') < text.index('ALIGN_REPLY=')
    assert text.index('verify_alignment following ||') < text.index("echo '启动完成：状态 RUNNING")
    subprocess.run(['bash', '-n', str(ROOT / 'scripts/run_bimanual_remote_feedback.sh')], check=True)
