#include "rcs_location/rcs_location_algorithm_imu.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace rcs_location;
namespace {
void near(double actual, double expected, double tolerance, const char *name) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
    throw std::runtime_error(std::string(name) + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
void check(bool condition, const char *name) { if (!condition) throw std::runtime_error(name); }
ImuOptions unfiltered() { ImuOptions options; options.deadband_enabled=false; return options; }
struct Fixture {
  ImuAlgorithm algorithm;
  tf2::Quaternion q;
  double time{0};
  Fixture(ImuOptions o = unfiltered(), double yaw = 0) : algorithm(o) {
    q.setRPY(0,0,yaw);
    for (int i=0;i<250;++i) step({0,0,0});
    check(algorithm.ready(), "initialization");
  }
  ImuResult step(tf2::Vector3 world_acceleration, tf2::Vector3 gyro = {0,0,0}) {
    time += 0.01;
    // 当前 LPMS 输出 = -(世界运动加速度 + 世界静止参考) 转回机体系。
    const auto measured = -tf2::quatRotate(q.inverse(), world_acceleration + tf2::Vector3(0,0,9.80665));
    return algorithm.update(time, q, measured, gyro);
  }
};
}
int main() {
 try {
  ImuAlgorithm recorded({});
  tf2::Quaternion recorded_q(-.004385461880424833,-.005297413845547106,.06616677117010697,.9977848783798877);
  for(int i=0;i<1000;++i) recorded.update(i*.01,recorded_q,
      {.17402285235049203,-.0944325891519431,-9.760720023474097},
      {.004790393259996355,-.00020496441314607093,.00422996403802848});
  check(recorded.ready(),"recorded LPMS initializes");
  near(recorded.estimate().x,0,1e-8,"recorded stationary x");
  near(recorded.estimate().y,0,1e-8,"recorded stationary y");
  // BE1 实测：驱动修正后的水平→倾斜静止，不能产生虚假平移。
  ImuAlgorithm measured_pose({});
  tf2::Quaternion flat(.0011994308201500395,.007390266179084221,.47354693863289926,.8807367294151943);
  tf2::Quaternion tilted_pose(.16166591972568198,.08866075846478132,.44985870872910727,.87385502393025);
  double measured_time=0;
  for(int i=0;i<250;++i) {
    measured_time+=.01;
    measured_pose.update(measured_time,flat,{.11154708748613607,-.08920998210262204,-9.807858052924432},{0,0,0});
  }
  for(int i=0;i<100;++i) {
    measured_time+=.01;
    measured_pose.update(measured_time,tilted_pose,{.08565740344488834,-3.555687492000794,-9.145882921877961},{0,0,0});
  }
  check(std::hypot(measured_pose.estimate().vx,measured_pose.estimate().vy)<.02,"measured tilt gravity velocity");
  check(std::hypot(measured_pose.estimate().x,measured_pose.estimate().y)<.02,"measured tilt gravity position");
  Fixture stationary;
  for(int i=0;i<1000;++i) stationary.step({0,0,0});
  near(stationary.algorithm.estimate().x,0,1e-9,"stationary x");
  near(stationary.algorithm.estimate().vy,0,1e-9,"stationary vy");
  Fixture moving;
  for(int i=0;i<200;++i) moving.step({1,0,0});
  const auto value = moving.algorithm.estimate();
  near(value.vx,1.995,0.001,"acceleration velocity");
  near(value.x,1.990025,0.001,"acceleration position");
  check(value.position_variance > 0 && value.velocity_variance > 0,"covariance grows");
  for(int i=0;i<100;++i) moving.step({0,0,0});
  check(moving.algorithm.estimate().vx > 1.99,"constant velocity is not stationary");
  Fixture rotated(unfiltered(),1.2);
  for(int i=0;i<200;++i) rotated.step({std::cos(1.2),std::sin(1.2),0});
  near(rotated.algorithm.estimate().x,value.x,1e-8,"initial heading origin");
  near(rotated.algorithm.estimate().y,0,1e-8,"initial heading y");
  Fixture tilted;
  tilted.q.setRPY(0.3,-0.2,0.6);
  for(int i=0;i<100;++i) tilted.step({0,0,0});
  near(tilted.algorithm.estimate().x,0,1e-9,"tilt gravity compensation");
  near(tilted.algorithm.estimate().yaw,0.6,1e-9,"yaw from quaternion");
  const auto before = moving.algorithm.estimate();
  moving.time += 1;
  check(moving.step({0,0,0}) == ImuResult::Gap,"gap rejected");
  near(moving.algorithm.estimate().x,before.x,1e-12,"no gap extrapolation");
  near(moving.algorithm.estimate().vx,0,1e-12,"gap velocity reset");
  check(moving.algorithm.estimate().position_variance > before.position_variance,"gap uncertainty");
  check(moving.algorithm.update(moving.time-1,moving.q,{0,0,-9.8},{0,0,0}) == ImuResult::Invalid,"old timestamp");
  const double nan=std::numeric_limits<double>::quiet_NaN();
  check(moving.algorithm.update(moving.time+0.01,moving.q,{nan,0,0},{0,0,0}) == ImuResult::Invalid,"NaN sample");
  moving.algorithm.reset();
  check(!moving.algorithm.ready(),"reset needs initialization");
  near(moving.algorithm.estimate().x,0,0,"reset position");
  for(int i=0;i<250;++i) moving.step({0,0,0});
  check(moving.algorithm.ready(),"reset reinitializes");
  ImuAlgorithm unsettled({}); tf2::Quaternion identity(0,0,0,1);
  for(int i=0;i<300;++i) unsettled.update(i*.01,identity,{0,0,-9.8},{0,0,0.2});
  check(!unsettled.ready(),"rotation prevents initialization");
  ImuOptions standard_options; standard_options.acceleration_sign=1;
  ImuAlgorithm standard(standard_options);
  for(int i=0;i<300;++i) standard.update(i*.01,identity,{0,0,9.80665},{0,0,0});
  check(standard.ready(),"standard specific force");
  near(standard.estimate().x,0,1e-9,"standard stationary");
  ImuOptions lever_options=unfiltered(); lever_options.imu_position={0.3,0,0};
  Fixture lever(lever_options);
  for(int i=0;i<200;++i) lever.step({0,0,0});
  near(lever.algorithm.estimate().x,0,1e-9,"lever stationary");
  for(int i=1;i<=300;++i) {
    const double t=i*.01, omega=t<1 ? .2*t : .2;
    const double alpha=t<=1 ? .2 : 0;
    const double theta=t<1 ? .1*t*t : .1+.2*(t-1);
    lever.q.setRPY(0,0,theta);
    lever.step({-.3*(std::cos(theta)*omega*omega+std::sin(theta)*alpha),
                 .3*(-std::sin(theta)*omega*omega+std::cos(theta)*alpha),0}, {0,0,omega});
  }
  near(lever.algorithm.estimate().x,0,.002,"lever rotating x");
  near(lever.algorithm.estimate().y,0,.002,"lever rotating y");
  near(lever.algorithm.estimate().vx,0,.002,"lever rotating vx");
  near(lever.algorithm.estimate().vy,0,.002,"lever rotating vy");
  ImuOptions filtered_options;
  filtered_options.zero_velocity_enabled=false;
  Fixture filtered(filtered_options);
  for(int i=0;i<1000;++i) filtered.step({.06*std::sin(i*.17),.08*std::cos(i*.23),0});
  near(filtered.algorithm.estimate().x,0,0,"deadband holds noisy stationary x");
  near(filtered.algorithm.estimate().y,0,0,"deadband holds noisy stationary y");
  near(filtered.algorithm.estimate().vx,0,0,"deadband holds noisy stationary velocity");
  for(int i=0;i<100;++i) filtered.step({.3,0,0});
  near(filtered.algorithm.estimate().vx,.199,.002,"soft deadband preserves larger acceleration");
  const auto filtered_speed=filtered.algorithm.estimate().vx;
  for(int i=0;i<100;++i) filtered.step({0,0,0});
  check(filtered.algorithm.estimate().vx>=filtered_speed,"deadband does not erase existing velocity");
  ImuOptions hold_options;
  Fixture held(hold_options);
  for(int i=0;i<100;++i) held.step({.3,0,0});
  for(int i=0;i<50;++i) held.step({0,0,0});
  check(held.algorithm.estimate().vx>.1,"zero velocity waits for quiet duration");
  for(int i=0;i<150;++i) held.step({0,0,0});
  near(held.algorithm.estimate().vx,0,0,"quiet duration clears accumulated velocity");
  const double held_x=held.algorithm.estimate().x;
  for(int i=0;i<1000;++i) held.step({.06*std::sin(i*.17),.08*std::cos(i*.23),0});
  near(held.algorithm.estimate().x,held_x,0,"quiet hold stops position drift");
  for(int i=0;i<7;++i) held.step({1,0,0});
  for(int i=0;i<25;++i) held.step({0,0,0});
  for(int i=0;i<4;++i) held.step({0,2,0});
  for(int i=0;i<100;++i) held.step({0,0,0});
  near(held.algorithm.estimate().x,held_x,0,"short pulses do not release quiet hold");
  near(held.algorithm.estimate().vy,0,0,"short pulses do not create velocity");
  for(int i=0;i<100;++i) held.step({.3,0,0});
  check(held.algorithm.estimate().vx>.1,"sustained acceleration releases quiet hold");
  ImuOptions adaptive_options;
  adaptive_options.acceleration_deadband={.001,.001,0};
  ImuAlgorithm adaptive(adaptive_options);
  for(int i=0;i<250;++i) adaptive.update(i*.01,identity,
      {-.02*std::sin(i*.27),-.03*std::cos(i*.21),-9.80665},{0,0,0});
  check(adaptive.ready(),"noise calibration initializes");
  check(adaptive.deadband().x()>.06 && adaptive.deadband().x()<.08,"adaptive x threshold");
  check(adaptive.deadband().y()>.09 && adaptive.deadband().y()<.12,"adaptive y threshold");
  for(int i=250;i<1000;++i) adaptive.update(i*.01,identity,
      {-.02*std::sin(i*.27),-.03*std::cos(i*.21),-9.80665},{0,0,0});
  near(adaptive.estimate().x,0,0,"adaptive stationary x");
  near(adaptive.estimate().vy,0,0,"adaptive stationary vy");
  adaptive.reset();
  near(adaptive.deadband().x(),.001,1e-12,"reset discards learned threshold");
  for(int i=0;i<250;++i) adaptive.update(i*.01,identity,{0,0,-9.80665},{0,0,0});
  near(adaptive.deadband().x(),.001,1e-12,"reset relearns quieter noise");
  bool rejected=false;
  try { ImuOptions bad; bad.max_dt=0; ImuAlgorithm invalid(bad); }
  catch(const std::invalid_argument&) { rejected=true; }
  check(rejected,"bad configuration");
  std::cout << "IMU algorithm: all trajectory and safety checks passed\n";
  return 0;
 } catch(const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
