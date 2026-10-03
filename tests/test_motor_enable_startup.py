"""No-motor regression for power-cycle detection and guarded startup recovery."""
import importlib.util
from pathlib import Path
import struct
import subprocess
import json
import tempfile
import unittest
import socket


ROOT = Path(__file__).parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


probe = load('check_motor_enable')
recovery = load('check_power_cycle_recovery')


class MotorEnableTest(unittest.TestCase):
    def packet(self, motor=7, state=1, fd=True, flags=1):
        data = bytes([(state << 4) | motor]) + bytes(7)
        if fd:
            return struct.pack('=IBB2x64s', motor + 0x10, 8, flags, data)
        return struct.pack('=IB3x8s', motor + 0x10, 8, data)

    def observations(self, state=1):
        return {(bus, motor): [(1., state), (1.1, state), (1.2, state)]
                for bus in ('can0', 'can1') for motor in range(1, 9)}

    def report(self, values):
        return probe.summarize(values, ['can0', 'can1'], 1.3)

    def test_fd_flags_are_not_motor_status(self):
        # candump ##107... and ##507... both mean data[0]=07, DISABLED.
        for flags in (1, 5):
            self.assertEqual(probe.decode_feedback(self.packet(state=0, flags=flags)), (7, 0))
        self.assertEqual(probe.decode_feedback(self.packet()), (7, 1))

    def test_classic_frames(self):
        self.assertEqual(probe.decode_feedback(self.packet(fd=False)), (7, 1))

    def test_kernel_timestamp_rejects_backlog_future_and_missing(self):
        cmsg=[(socket.SOL_SOCKET,probe.SO_TIMESTAMPNS,struct.pack('=qq',10,0))]
        self.assertAlmostEqual(probe.feedback_receive_time(cmsg,0,10_100_000_000,20.),19.9)
        self.assertIsNone(probe.feedback_receive_time(cmsg,0,11_000_000_000,20.))
        self.assertIsNone(probe.feedback_receive_time(cmsg,0,9_999_999_999,20.))
        self.assertIsNone(probe.feedback_receive_time([],0,10_100_000_000,20.))
        self.assertIsNone(probe.feedback_receive_time(cmsg,socket.MSG_CTRUNC,10_100_000_000,20.))

    def test_malformed_unrelated_and_extended_frames_ignored(self):
        for packet in (b'', bytes(72), self.packet()[:15]):
            self.assertIsNone(probe.decode_feedback(packet))
        for can_id in (7, 0x7FF, 0x80000017, 0x40000017, 0x20000017, 0x16):
            data = bytearray(self.packet())
            struct.pack_into('=I', data, 0, can_id)
            self.assertIsNone(probe.decode_feedback(data))

    def test_all_enabled(self):
        self.assertEqual(self.report(self.observations())['result'], 'ENABLED')

    def test_power_cycle_even_with_zero_position_error(self):
        self.assertEqual(self.report(self.observations(0))['result'], 'DISABLED')

    def test_partial_disable_not_power_cycle(self):
        values = self.observations()
        values['can1', 7] = [(1., 0), (1.1, 0), (1.2, 0)]
        self.assertEqual(self.report(values)['result'], 'MIXED')

    def test_missing_stale_sparse_or_transition_rejected(self):
        for replacement in ([], [(0., 1)] * 3, [(1.2, 1)], [(1., 0), (1.1, 1), (1.2, 1)]):
            values = self.observations()
            values['can1', 7] = replacement
            self.assertEqual(self.report(values)['result'], 'UNKNOWN')

    def test_motor_fault_is_not_disabled(self):
        self.assertEqual(self.report(self.observations(8))['result'], 'FAULT')

    def test_both_computers_must_agree(self):
        for state in ('ENABLED', 'DISABLED'):
            self.assertEqual(probe.combined({'result': state}, {'result': state})['result'], state)
        self.assertEqual(probe.combined({'result': 'ENABLED'}, {'result': 'DISABLED'})['result'], 'MIXED')
        self.assertEqual(probe.combined({'result': 'ENABLED'}, {'result': 'UNKNOWN'})['result'], 'UNKNOWN')

    def healthy(self):
        return {'state': 'RUNNING', 'fault_bits': 0, 'feedback_fresh_for_control': True,
                'enabled_arms': ['left', 'right'], 'collection': {
                    'left_mode': 'FOLLOW', 'right_mode': 'FOLLOW', 'recording': None,
                    'return_phase': 'idle', 'transitioning_arms': []}}

    def test_only_all_disabled_idle_case_is_recoverable(self):
        self.assertTrue(recovery.can_reinitialize(self.healthy(), {'result': 'DISABLED'}))
        for state in ('ENABLED', 'MIXED', 'FAULT', 'UNKNOWN', None):
            self.assertFalse(recovery.can_reinitialize(self.healthy(), {'result': state}))

    def test_fault_estop_missing_feedback_never_auto_restart(self):
        for field, value in (('state', 'FAULT'), ('state', 'E_STOP'), ('fault_bits', 8),
                             ('feedback_fresh_for_control', False), ('enabled_arms', ['left'])):
            state = self.healthy(); state[field] = value
            self.assertFalse(recovery.can_reinitialize(state, {'result': 'DISABLED'}))

    def test_hold_return_recording_must_not_rehome(self):
        for field, value in (('left_mode', 'HOLD'), ('right_mode', 'RETURNING'),
                             ('return_phase', 'moving'), ('transitioning_arms', ['left']),
                             ('recording', {'token': 'active'})):
            state = self.healthy(); state['collection'][field] = value
            self.assertFalse(recovery.can_reinitialize(state, {'result': 'DISABLED'}))
        self.assertFalse(recovery.can_reinitialize({}, {'result': 'DISABLED'}))

    def test_startup_shell_syntax(self):
        for name in ('daily_start_teleop_rgbd.sh', 'run_bimanual_remote_feedback.sh',
                     'daily_start_teleop_only.sh'):
            subprocess.run(['bash', '-n', str(ROOT / 'scripts' / name)], check=True)

    def test_probe_source_contains_no_can_transmit(self):
        source = (ROOT / 'scripts/check_motor_enable.py').read_text()
        self.assertNotIn('.send(', source)
        self.assertNotIn('.sendto(', source)

    def test_real_shell_reuse_confirmed_recovery_and_rejection_paths(self):
        # Exercise the actual launcher branch, replacing hardware/SSH/systemd
        # boundaries only. No shell in this test can touch real robot services.
        source = (ROOT / 'scripts/daily_start_teleop_rgbd.sh').read_text()
        branch = source[source.index('\nensure_teleop_service\nstatus='):
                        source.index("\necho '  正在核实本会话两端")]
        # Feed simulated operator input, never access a real controlling TTY.
        branch = branch.replace('</dev/tty', '</dev/stdin')
        branch = branch.replace(
            '/usr/bin/python3 "$TELEOP_ROOT/scripts/check_motor_enable.py" --jetson "$JETSON_HOST"',
            'printf "%s" "$MOTOR_FIXTURE"')
        setup = r'''
set -euo pipefail
TELEOP_ROOT="$1"; LOG_DIR="$2"; JETSON_HOST=mock; TELEOP_SERVICE=mock
STATUS_FIXTURE="$3"; MOTOR_FIXTURE="$4"
ensure_teleop_service() { :; }
say() { printf '%s\n' "$*"; }
teleop_status() { printf '%s' "$STATUS_FIXTURE"; }
refresh_healthy_running_status() { return 0; }
status_is_safe_to_reuse() { return 0; }
recover_restarted_peer() { return 1; }
status_is_healthy_running() { return 0; }
ssh() { command cat >/dev/null; return 0; }
sleep() { :; }
systemctl() {
  case "$*" in
    *restart*) echo MOCK_RESTART; printf '启动完成：状态 RUNNING\n' >"$LOG_DIR/teleop.log" ;;
    *show*) echo 12345 ;;
  esac
  return 0
}
'''
        for motor_state, answer, expected_restart, expected_exit in (
                ('ENABLED', '', False, 0), ('DISABLED', 'r\n', True, 0),
                ('DISABLED', '', False, 2), ('DISABLED', '不恢复\n', False, 2), ('MIXED', '启动\n', False, 2),
                ('UNKNOWN', '启动\n', False, 2)):
            with self.subTest(motor_state=motor_state, answer=answer), tempfile.TemporaryDirectory() as tmp:
                result = subprocess.run(
                    ['bash', '-c', setup + branch, 'test', str(ROOT), tmp,
                     json.dumps(self.healthy()), json.dumps({'result': motor_state})],
                    input=answer, text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, expected_exit, result.stderr)
                self.assertEqual('MOCK_RESTART' in result.stdout, expected_restart, result.stdout)

        # Reproduce the reported old timeout + all-disabled + retained HOLD.
        # Only a successful explicit recovery gate may reach the normal restart.
        fault = self.healthy()
        fault.update(state='FAULT', fault_bits=1, reason='leader action receive timeout')
        fault['collection']['left_mode'] = 'HOLD'
        for gate_result in (0, 1, 2, 3):
            with self.subTest(recovery_gate=gate_result), tempfile.TemporaryDirectory() as tmp:
                fixture_setup = setup.replace('recover_restarted_peer() { return 1; }',
                    f'recover_restarted_peer() {{ return {gate_result}; }}')
                result = subprocess.run(
                    ['bash', '-c', fixture_setup + branch, 'test', str(ROOT), tmp,
                     json.dumps(fault), json.dumps({'result': 'DISABLED'})],
                    input='', text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 0 if gate_result == 0 else 2,
                                 result.stdout + result.stderr)
                self.assertEqual('MOCK_RESTART' in result.stdout, gate_result == 0)


if __name__ == '__main__':
    unittest.main()
