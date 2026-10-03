// Hardware-free plant approximation using the production correction function.
#include "openarm_gravity_pd_control/tracking_assist.hpp"
#include "openarm_gravity_pd_control/drive_feedback_guard.hpp"
#include <cassert>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <thread>
using namespace openarm_gravity_pd_control;

struct Result { double error, q, peak_assist, settled_s; };
Result simulate(double kp, double kd, double extra_kp, double extra_kd, double cap,
                double load, double inertia, double start, bool enabled,
                bool blocked=false, double friction=0., bool sinusoid=false) {
  double q=start, v=0., command=start, peak=0., stable=0., settled=-1.;
  const double dt=.002, goal=.6283185307179586;
  for(int step=0;step<10000;++step) {
    double t=step*dt;
    const double target=sinusoid && t>3 ? goal+.1*std::sin((t-3)*.7) :
      start+(goal-start)*std::min(t/2.,1.);
    command=rateLimitedPosition(command,target,.8,dt);
    const double assist=trackingAssist(command-q,v,extra_kp,extra_kd,cap,enabled);
    peak=std::max(peak,std::abs(assist));
    // Residual load = actual gravity minus modeled feedforward. PD acts in
    // motor firmware; simulation omits its bandwidth/quantization/flexibility.
    double drive=kp*(command-q)-kd*v+assist-load;
    if (std::abs(v)<1e-5 && std::abs(drive)<=friction) drive=0;
    else drive-=friction*(v>1e-5?1:(v<-1e-5?-1:(drive>=0?1:-1)));
    if (!blocked) { v+=dt*(drive-.15*v)/inertia; q+=dt*v; }
    stable = t>=2. && std::abs(target-q)<.05 && std::abs(v)<.005 ? stable+dt : 0.;
    if (stable>=1. && settled<0.) settled=t;
    assert(std::isfinite(q)); assert(std::abs(assist)<=cap+1e-10);
  }
  return {goal-q,q,peak,settled};
}

int main(int argc, char** argv) {
  if (argc==2) {
    // Explicit optional diagnostic: listens only, no CAN transmission.
    DriveFeedbackGuard passive; passive.open(argv[1]);
    int healthy=0;
    for(int i=0;i<200;++i) {
      healthy+=passive.poll()?1:0;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout<<"{\"readonly\":true,\"healthy_samples\":"<<healthy<<",\"samples\":200}\n";
    return 0;
  }
  using C=DriveFeedbackGuard::Clock;
  for(bool fresh:{false,true}) for(bool fault:{false,true}) for(bool enabled:{false,true}) {
    assert(collectionReturnAssistAllowed(fresh,fault,enabled)==(fresh&&!fault&&enabled));
  }
  assert(kCollectionReturnMaxVelocity==.20);
  DriveFeedbackGuard guard;
  auto time=C::now();
  assert(!guard.healthy(time));
  for(int j=1;j<=7;++j) {
    canfd_frame f{}; f.can_id=0x10+j; f.len=8; f.flags=5; f.data[0]=0x10|j;
    guard.observe(f,CANFD_MTU,time);
  }
  assert(guard.healthy(time));
  assert(!guard.healthy(time+std::chrono::milliseconds(51)));
  canfd_frame bad{};bad.can_id=0x17;bad.len=8;bad.data[0]=7;
  guard.observe(bad,CANFD_MTU,time);assert(!guard.healthy(time));
  bad.data[0]=0x87;guard.observe(bad,CAN_MTU,time);assert(!guard.healthy(time));
  bad.data[0]=0x17;guard.observe(bad,CAN_MTU,time);assert(guard.healthy(time));
  for(double e:{-10.,-.2,0.,.2,10.}) for(double v:{-2.,0.,2.}) {
    assert(std::abs(trackingAssist(e,v,35,.6,1.5,true))<=1.5);
    assert(trackingAssist(e,v,35,.6,1.5,false)==0.);
  }
  assert(trackingAssist(std::numeric_limits<double>::quiet_NaN(),0,35,.6,1.5,true)==0);
  // Long hard contact stores no torque history: release/reset error -> zero.
  for(int i=0;i<100000;++i) assert(trackingAssist(.2,0,35,.6,1.5,true)==1.5);
  assert(trackingAssist(0,0,35,.6,1.5,true)==0);
  assert(trackingAssist(.2,0,35,.6,1.5,false)==0); // stale/disabled gate

  const auto old_j4=simulate(15,1.4,35,.6,1.5,15*.114,.12,-.2,false);
  const auto new_j4=simulate(15,1.4,35,.6,1.5,15*.114,.12,-.2,true);
  const auto old_j7=simulate(10,.5,25,.3,.8,10*.121,.025,.4,false);
  const auto new_j7=simulate(20,.7,25,.3,.8,10*.121,.025,.4,true);
  assert(std::abs(old_j4.error-.114)<.002);
  assert(std::abs(new_j4.error)<.05);
  assert(std::abs(old_j7.error-.121)<.002);
  assert(std::abs(new_j7.error)<.035);
  assert(new_j4.settled_s>0 && new_j4.settled_s<15);
  assert(new_j7.settled_s>0 && new_j7.settled_s<15);
  // Measured return failure: leader J3/J6 stop short with startup gains alone.
  // Same production limited assist used in explicit return reduces residuals,
  // without learning offsets or increasing the alignment tolerance.
  const auto return_j3=simulate(15,1.4,20,.4,1.,15*.119,.12,0.,true);
  const auto return_j6=simulate(5,.4,12,.2,.45,5*.085,.025,0.,true);
  assert(std::abs(return_j3.error)<.06);
  assert(std::abs(return_j6.error)<.06);
  // Different starts, inertia, gravity residuals, static friction and lag.
  int cases=0; double worst_settle=0.;
  for(double start:{-.6,0.,.8,1.2}) for(double inertia:{.04,.12,.35})
    for(double load:{-1.71,0.,1.71}) for(double friction:{0.,.15}) {
      auto r=simulate(15,1.4,35,.6,1.5,load,inertia,start,true,false,friction);
      assert(std::abs(r.error)<.05);
      assert(r.settled_s>0 && r.settled_s<15);
      worst_settle=std::max(worst_settle,r.settled_s); ++cases;
    }
  auto blocked=simulate(15,1.4,35,.6,1.5,1.71,.12,0.,true,true);
  assert(std::abs(blocked.error)>.07); // startup must fail, not "calibrate" away blockage
  auto overload=simulate(15,1.4,35,.6,1.5,4.,.12,0.,true);
  assert(std::abs(overload.error)>.07); // cap prevents unlimited correction
  simulate(15,1.4,35,.6,1.5,1.71,.12,-.2,true,false,0.,true);
  std::cout<<std::setprecision(9)
    <<"{\"old_j4_rad\":"<<old_j4.error<<",\"new_j4_rad\":"<<new_j4.error
    <<",\"old_j7_rad\":"<<old_j7.error<<",\"new_j7_rad\":"<<new_j7.error
    <<",\"bounded_contact_error_rad\":"<<blocked.error
    <<",\"overload_error_rad\":"<<overload.error<<",\"sweep_cases\":"<<cases
    <<",\"worst_settle_s\":"<<worst_settle<<"}\n";
}
