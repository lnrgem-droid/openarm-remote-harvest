"""Synthetic scheduler/UDP regression: no hardware or production sockets."""
from collections import deque
from types import SimpleNamespace
import importlib.util
import pytest
from remote_teleop_protocol import ActionCommand, encode_action
from remote_teleop_runtime import follower_io as current

class Clock:
    def __init__(self):self.ns=1_000_000_000
    def __call__(self):return self.ns
    def advance(self,ms):self.ns+=int(ms*1e6)
class Socket:
    def __init__(self,clock):self.queue=deque();self.clock=clock;self.sent=[];self.cost=0
    def recvfrom(self,_):
        if not self.queue:raise BlockingIOError
        self.clock.advance(self.cost)
        return self.queue.popleft(),('leader',50010)
    def recv(self,size):return self.recvfrom(size)[0]
    def sendto(self,data,peer):self.sent.append(data)

def packet(seq):return encode_action(ActionCommand(88,seq,seq*4_000_000,(0.,)*16,100_000_000))
def rig(module=current):
    clock=Clock();udp=Socket(clock);unix=Socket(clock)
    args={'clock':clock}
    if 'queue_empty_probe' in module.FollowerIOWorker.__init__.__code__.co_varnames:
        args['queue_empty_probe']=lambda:not udp.queue
    worker=module.FollowerIOWorker(udp,SimpleNamespace(sock=unix,server='offline'),77,**args)
    return worker,clock,udp,unix

def test_receive_1_2ms_scheduler_delay_cannot_starve_real_actions():
    worker,clock,udp,_=rig();worker.step();initial=clock()
    for seq in range(1,61):
        clock.advance(4);udp.cost=1.2;udp.queue.append(packet(seq));worker.step()
        sample=worker.snapshot()['action']
        assert sample is not None, 'Fresh arriving actions discarded only because recv exhausted 1ms budget'
        assert sample.action.sequence==seq
        assert sample.safe_rx_ns < sample.processed_ns
        assert clock()-sample.safe_rx_ns<20_000_000
    assert clock()-initial>150_000_000

def test_budget_yield_keeps_private_candidate_but_never_partial_backlog():
    worker,clock,udp,_=rig();worker.step();boundary=clock();clock.advance(4)
    udp.queue.extend(packet(i) for i in range(1,66))
    worker.step();assert worker.snapshot()['action'] is None
    worker.step();assert worker.snapshot()['action'] is None
    worker.step();sample=worker.snapshot()['action']
    assert sample.action.sequence==65 and sample.safe_rx_ns==boundary
    assert worker.diagnostics['rx_budget_yields']>=2

def test_old_backlog_after_budget_yield_never_gets_receive_time_as_freshness():
    worker,clock,udp,_=rig();worker.step();clock.advance(4)
    udp.queue.extend(packet(i) for i in range(1,66));worker.step()
    clock.advance(151);worker.step();worker.step()
    assert worker.snapshot()['action'] is None
    clock.advance(4);udp.queue.append(packet(66));worker.step()
    assert worker.snapshot()['action'].action.sequence==66

def test_epoch_change_cannot_commit_carried_candidate():
    worker,clock,udp,_=rig();worker.step();clock.advance(4)
    udp.queue.extend(packet(i) for i in range(1,34));worker.step()
    worker.begin_command();udp.queue.clear();worker.step()
    assert worker.snapshot()['action'] is None

def test_probe_preemption_never_moves_empty_boundary_to_after_arrival():
    worker,clock,udp,_=rig();worker.step();clock.advance(4);udp.cost=1.2;udp.queue.append(packet(1))
    before_probe=[]
    def probe():
        before_probe.append(clock());clock.advance(90)
        return True
    worker.queue_empty_probe=probe;worker.step()
    assert worker.empty_boundary_ns==before_probe[0]
    if worker.snapshot()['action'] is not None:
        assert clock()-worker.snapshot()['action'].safe_rx_ns>=90_000_000
    # A >100ms-old candidate must not publish even if the probe proves empty.
    clock.advance(4);udp.queue.append(packet(2))
    worker.queue_empty_probe=lambda:(clock.advance(110) or True);previous=worker.snapshot()['action'];worker.step()
    assert worker.snapshot()['action'] is previous

@pytest.mark.parametrize('delay_ms',[5,10,19])
def test_scheduled_local_reply_preserves_original_age_without_false_outage(delay_ms):
    import json
    worker,clock,udp,unix=rig();worker.control_completed(clock(),clock(),clock(),True);worker.step()
    pending=dict(worker.pending_heartbeat);sent=pending['sent_ns']
    clock.advance(delay_ms)
    reply={'state':'RUNNING','fault_bits':0,'watchdog_session_id':99,'snapshot_monotonic_ns':clock(),'reply_to':pending['reply_to']}
    unix.queue.append(json.dumps(reply).encode());worker.step()
    assert worker.snapshot()['safety']['state']=='RUNNING'
    assert worker.snapshot()['safety_rx_ns']==sent  # Never the late read time.
    assert clock()-worker.snapshot()['safety_rx_ns']==int(delay_ms*1e6)


def test_reply_after_new_budget_still_cannot_grant_authority():
    import json
    worker,clock,udp,unix=rig();worker.control_completed(clock(),clock(),clock(),True);worker.step()
    pending=dict(worker.pending_heartbeat);clock.advance(21)
    unix.queue.append(json.dumps({'state':'RUNNING','fault_bits':0,'watchdog_session_id':99,'snapshot_monotonic_ns':clock(),'reply_to':pending['reply_to']}).encode())
    worker.step()
    assert worker.snapshot()['safety_rx_ns']==0
    assert worker.snapshot()['diagnostics']['heartbeat_timeouts']==1
