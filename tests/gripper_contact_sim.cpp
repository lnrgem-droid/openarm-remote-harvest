#include "openarm_gravity_pd_control/gripper_contact.hpp"
#include "openarm_gravity_pd_control/drive_feedback_guard.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace openarm_gravity_pd_control;
int main() {
  GripperContact h;
  // Freely following, quantization/deadband, and remote more closed: no pull.
  assert(h.step(-.5,0,-.5,-1.0472,.002,true)==0);
  assert(h.step(-.495,0,-.5,-1.0472,.002,true)==0);
  assert(h.step(-.6,0,-.5,-1.0472,.002,true)==0);
  double previous=0.;
  for(int i=0;i<150;++i) {
    const double t=h.step(-.4,0,-.5,-1.0472,.002,true);
    assert(t<=0 && t>=-.4 && std::abs(t-previous)<=.004000001);
    previous=t;
  }
  assert(std::abs(previous+.36)<1e-9);
  const double reflected=previous;
  for(int i=0;i<50000;++i) assert(h.step(-.1,0,-.9,-1.0472,.002,true)>=-.4);
  assert(h.step(-.104,-.1,-.9,-1.0472,.002,true)==-.4); // helps opening, never closes
  assert(h.step(-.104,0,-.9,-1.0472,.002,true)==-.4); // contact remains at rest
  h.step(-.1,0,-.9,-1.0472,.002,true);
  assert(h.step(-.1,0,-.9,-1.0472,.002,false)==0); // lost permit/freshness
  assert(h.step(-.1,0,-.9,-1.0472,.002,true)==-.004); // no stored preload
  assert(h.step(-.5,0,-.5,-1.0472,.002,true)==0); // no lingering contact
  assert(h.step(-.1,0,-2.,-1.0472,.002,true)==0);
  assert(h.step(-.1,0,-.9,-1.0472,.1,true)==0);
  assert(h.step(std::numeric_limits<double>::quiet_NaN(),0,-.9,-1.0472,.002,true)==0);
  // Stationary position with the actual quantized velocity values must not
  // suppress contact feedback. Also tolerate +/- one encoder count of q.
  GripperContact jitter;
  double reflected_jitter=0.;
  for(int i=0;i<500;++i) {
    const double q=-.4+(i%2 ? .0003814755 : -.0003814755);
    reflected_jitter=jitter.step(q,i%3==2 ? -.021978021978 : -.007326007326,
                                 -.5,-1.0472,.002,true);
    assert(reflected_jitter<=0 && reflected_jitter>=-.4);
  }
  assert(reflected_jitter<-.35);
  // Tiny opening while still blocked must not latch away all resistance.
  for(int i=1;i<=40;++i) jitter.step(-.4-i*.0001,-.01,-.5,-1.0472,.002,true);
  for(int i=0;i<100;++i) assert(std::abs(jitter.step(-.404,0,-.5,-1.0472,.002,true)+.344)<1e-9);
  // Actual opening up to/beyond the remote opening has zero contact cue.
  assert(jitter.step(-.495,-.1,-.5,-1.0472,.002,true)==0);
  assert(jitter.step(-.6,-.1,-.5,-1.0472,.002,true)==0);
  // Closing again ramps, never jumps to full contact torque.
  assert(jitter.step(-.400,0.1,-.5,-1.0472,.002,true)==-.004);
  // Invalid/stale permit discards force memory.
  assert(jitter.step(-.400,0,-.5,-1.0472,.002,false)==0);
  assert(jitter.step(-.400,0,-.5,-1.0472,.002,true)==-.004);
  // Recorded field geometry: leader -0.571641, follower -0.875296.
  // The old direction latch yielded zero indefinitely after slight opening.
  GripperContact field;
  for(int i=0;i<200;++i) field.step(-.567,0,-.8752956435,-1.0472,.002,true);
  for(int i=0;i<200;++i)
    assert(field.step(-.5716411078,-.0073260073,-.8752956435,-1.0472,.002,true)==-.4);
  // Across travel and either velocity sign the spring cannot close the hand.
  for(int i=0;i<=100;++i) for(int j=0;j<=100;++j) {
    const double local=-1.0472*i/100.,remote=-1.0472*j/100.;
    const double t=field.step(local,(i%2 ? -.03 : .03),remote,-1.0472,.002,true);
    assert(t>=-.4 && t<=0.);
    if(local<=remote+.01) assert(t==0.);
  }
  // Guard must accept the actual enabled motor 8, not arm-only health.
  DriveFeedbackGuard g;
  auto now=DriveFeedbackGuard::Clock::now();
  assert(!g.gripperHealthy(now));
  canfd_frame f{}; f.can_id=0x18; f.len=8; f.data[0]=0x18;
  g.observe(f,CANFD_MTU,now); assert(g.gripperHealthy(now));
  assert(!g.gripperHealthy(now+std::chrono::milliseconds(51)));
  f.data[0]=8;g.observe(f,CANFD_MTU,now);assert(!g.gripperHealthy(now));
  f.data[0]=0x88;g.observe(f,CAN_MTU,now);assert(!g.gripperHealthy(now));
  std::cout<<"{\"gap_rad\":0.1,\"old_torque_nm\":-0.08,\"new_torque_nm\":"<<reflected
    <<",\"cap_nm\":0.4,\"opening_torque_nm\":0,\"stale_torque_nm\":0}\n";
}
