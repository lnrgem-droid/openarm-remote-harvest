"""Real loopback UDP + isolated watchdog, injected receive scheduling delay."""
import time
from test_follower_io_integration import LocalRig
from remote_teleop_protocol import FaultBits

class SlowReceive:
    def __init__(self,sock):self.sock=sock
    def recvfrom(self,size):
        value=self.sock.recvfrom(size)
        time.sleep(.0012)  # Beyond the unchanged 1ms batch work budget.
        return value
    def fileno(self):return self.sock.fileno()
    def close(self):return self.sock.close()


def test_three_capture_length_rounds_keep_fresh_udp_but_real_loss_still_faults(tmp_path):
    rig=LocalRig(tmp_path)
    try:
        rig.run()
        rig.worker.udp=SlowReceive(rig.rx)
        for _ in range(3):
            state=rig.pump(.7)
            assert state['safety']['state']=='RUNNING',state
            assert not state['safety']['fault_bits']
            assert state['diagnostics']['rx_budget_yields']>0
            assert state['diagnostics']['worker_error'] is None
        state=rig.pump(.20,actions=False)
        assert state['safety']['state']=='FAULT'
        assert state['safety']['fault_bits'] & int(FaultBits.NETWORK_TIMEOUT)
        assert state['safety']['first_fault']['action_age_ms']>150
        assert rig.pump(.1)['safety']['state']=='FAULT'  # No auto recovery.
    finally:rig.close()
