#include <iostream>
#include <limits>
#include "rcs_imu/rcs_imu_conversion.hpp"
void check(bool condition) { if (!condition) throw std::runtime_error("conversion test failed"); }
bool near(double a,double b) { return std::abs(a-b)<1e-5; }
int main() {
    IG1ImuDataI data; data.reset();
    IG1SettingsI settings; settings.reset();
    tf2::Quaternion mount; mount.setRPY(0,0,0);
    auto missing=rcs_imu::convert(data,settings,mount);
    check(missing.orientation_covariance[0]==-1 && missing.linear_acceleration_covariance[0]==-1);
    settings.transmitDataConfig=TDR_QUAT_OUTPUT_ENABLED | TDR_GYR1_ALIGN_CALIBRATED_OUTPUT_ENABLED | TDR_ACC_CALIBRATED_OUTPUT_ENABLED;
    data.quaternion.data[0]=2.0;
    data.gyroIIAlignmentCalibrated.data[0]=180;
    data.accCalibrated.data[2]=-1;
    auto out=rcs_imu::convert(data,settings,mount);
    check(near(out.orientation.w,1) && near(out.angular_velocity.x,std::acos(-1.0)));
    check(near(out.linear_acceleration.z,9.80665));
    settings.useRadianOutput=1;
    data.gyroIIAlignmentCalibrated.data[0]=1;
    mount.setRPY(0,0,std::acos(-1.0)/2);
    out=rcs_imu::convert(data,settings,mount);
    check(near(out.angular_velocity.x,0) && near(out.angular_velocity.y,1));
    check(near(out.orientation.z,-std::sqrt(0.5)));
    data.quaternion.data[0]=0;
    bool rejected=false;
    try { rcs_imu::convert(data,settings,mount); } catch (const std::runtime_error &) { rejected=true; }
    check(rejected);
    data.quaternion.data[0]=1;
    data.accCalibrated.data[0]=std::numeric_limits<float>::quiet_NaN();
    rejected=false;
    try { rcs_imu::convert(data,settings,mount); } catch (const std::runtime_error &) { rejected=true; }
    check(rejected);
    std::cout << "PASS: units, mount rotation, quaternion, missing fields, invalid data\n";
}
