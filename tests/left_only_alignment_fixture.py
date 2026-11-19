"""Shared synthetic left-only alignment fixtures; no robot access."""
import importlib.util
import json
from pathlib import Path

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
