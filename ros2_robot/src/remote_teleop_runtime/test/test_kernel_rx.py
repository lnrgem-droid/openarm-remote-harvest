import importlib.util
import socket
import struct
import time
from types import SimpleNamespace
import pytest
from remote_teleop_runtime.kernel_rx import KernelReceiveClock, SO_TIMESTAMPNS_NEW, CLOCK_GUARD_NS
from remote_teleop_runtime.follower_io import FollowerIOWorker
from test_action_receive_budget import rig, packet

class Clocks:
    def __init__(self):self.mono=10_000_000_000;self.offset=1_790_000_000_000_000_000
    def m(self):return self.mono
    def wall(self):return self.mono+self.offset
    def stamp(self, age=0):
        s,n=divmod(self.wall()-age,1_000_000_000)
        return [(socket.SOL_SOCKET,SO_TIMESTAMPNS_NEW,struct.pack('=qq',s,n))]
class ConfigSocket:
    def setsockopt(self,*args):self.config=args

def test_fixed_conversion_preserves_actual_packet_age():
    c=Clocks();k=KernelReceiveClock(ConfigSocket(),monotonic=c.m,realtime=c.wall)
    old=c.stamp();c.mono+=180_000_000
    assert c.m()-k.timestamp(old,0)==181_000_000
    assert c.m()-k.timestamp(c.stamp(),0)==CLOCK_GUARD_NS

@pytest.mark.parametrize('step',[2_000_000,-2_000_000,1_000_000_000,-1_000_000_000])
def test_wall_clock_step_fails_closed_without_rebasing_old_packets(step):
    c=Clocks();k=KernelReceiveClock(ConfigSocket(),monotonic=c.m,realtime=c.wall)
    old=c.stamp();c.offset+=step
    with pytest.raises(RuntimeError,match='clock changed'):k.timestamp(old,0)

@pytest.mark.parametrize('step',[500_000,-500_000])
def test_small_wall_adjustment_never_renews_a_packet(step):
    c=Clocks();k=KernelReceiveClock(ConfigSocket(),monotonic=c.m,realtime=c.wall)
    old=c.stamp();before=k.timestamp(old,0);c.mono+=2_000_000;c.offset+=step
    assert k.timestamp(old,0)==before
    assert k.timestamp(c.stamp(),0)<=c.m()

@pytest.mark.parametrize('flags',[socket.MSG_TRUNC,socket.MSG_CTRUNC])
def test_truncation_fails_closed(flags):
    c=Clocks();k=KernelReceiveClock(ConfigSocket(),monotonic=c.m,realtime=c.wall)
    assert k.timestamp(c.stamp(),flags) is None

def test_missing_invalid_and_future_stamp_rejected():
    c=Clocks();k=KernelReceiveClock(ConfigSocket(),monotonic=c.m,realtime=c.wall)
    assert k.timestamp([],0) is None
    assert k.timestamp(c.stamp(-1),0) is None
    assert k.timestamp(c.stamp()+c.stamp(),0) is None
    assert k.timestamp([(socket.SOL_SOCKET,SO_TIMESTAMPNS_NEW,b'bad')],0) is None

class BusySocket:
    """Continuous arrivals at a fixed small age; recv never reports EAGAIN."""
    def __init__(self,clock):self.clock=clock;self.seq=0;self.age=4_000_000;self.callback=None
    def recvfrom(self,size):
        self.seq+=1;self.clock.advance(1.2)
        if self.callback:self.callback()
        return packet(self.seq),('127.0.0.1',40000)
    def recvmsg(self,size,space):
        data,peer=self.recvfrom(size)
        return data,self.clock()-self.age,0,peer

def busy(kernel=True):
    w,c,_,_=rig();w.step();sock=BusySocket(c);w.udp=sock;w.queue_empty_probe=lambda:False
    if kernel:w.kernel_clock=SimpleNamespace(timestamp=lambda ancillary,flags:ancillary)
    c.advance(10)
    return w,c,sock

def test_continuously_nonempty_fresh_queue_reproduces_old_starvation():
    w,c,s=busy(False)
    for _ in range(80):c.advance(4);w.step()
    assert w.action is None and w.diagnostics['rx_packets']==80

def test_timestamped_fresh_queue_can_publish_at_budget_without_empty_queue():
    w,c,s=busy()
    for _ in range(80):
        c.advance(4);w.step()
        assert w.action.action.sequence==s.seq
        assert c()-w.action.safe_rx_ns==s.age
    assert w.diagnostics['rx_kernel_committed']==80

def test_old_queue_cannot_be_renewed_and_does_not_replace_valid_action():
    w,c,s=busy();w.step();prev=w.action;s.age=101_000_000
    for _ in range(50):c.advance(4);w.step()
    assert w.action is prev and c()-prev.safe_rx_ns>150_000_000
    assert w.diagnostics['rx_kernel_stale']==50

def test_pre_command_candidate_cannot_cross_epoch():
    w,c,s=busy();s.callback=w.begin_command;w.step();assert w.action is None

def test_reset_rejects_packets_that_arrived_before_reset():
    w,c,s=busy();w.step();c.advance(30);w.kernel_min_rx_ns=c();w.action=None;s.age=40_000_000
    w.step();assert w.action is None
    s.age=2_000_000;c.advance(5);w.step();assert w.action is not None

def test_real_kernel_packet_age_survives_queued_delay():
    c=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);tx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        c.bind(('127.0.0.1',0));c.settimeout(1);k=KernelReceiveClock(c)
        tx.sendto(b'queued',c.getsockname());time.sleep(.12)
        data,anc,flags,_=c.recvmsg(1024,socket.CMSG_SPACE(16))
        assert data==b'queued'
        assert time.monotonic_ns()-k.timestamp(anc,flags)>=120_000_000
        tx.sendto(b'fresh',c.getsockname());data,anc,flags,_=c.recvmsg(1024,socket.CMSG_SPACE(16))
        assert 0<=time.monotonic_ns()-k.timestamp(anc,flags)<20_000_000
    finally:c.close();tx.close()
