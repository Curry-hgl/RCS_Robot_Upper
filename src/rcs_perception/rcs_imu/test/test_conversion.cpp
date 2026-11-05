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
    // 两组 BE1 实测静止数据：水平、倾斜约 21°。
    const double recorded[2][7] = {
        {-0.0011994308201500395,-0.007390266179084221,-0.47354693863289926,0.8807367294151943,
         0.11154708748613607,-0.08920998210262204,-9.807858052924432},
        {-0.16166591972568198,-0.08866075846478132,-0.44985870872910727,0.87385502393025,
         0.08565740344488834,-3.555687492000794,-9.145882921877961}
    };
    for (const auto &sample : recorded) {
        // 历史消息曾错误取逆，恢复 SDK 原始四元数。
        data.quaternion.data[0]=sample[3];
        for (int i=0;i<3;++i) {
            data.quaternion.data[i+1]=-sample[i];
            data.accCalibrated.data[i]=-sample[i+4]/9.80665;
        }
        for (bool mounted : {false,true}) {
            mount.setRPY(mounted ? 0.3 : 0, mounted ? -0.2 : 0, mounted ? 0.4 : 0);
            out=rcs_imu::convert(data,settings,mount);
            tf2::Quaternion q(out.orientation.x,out.orientation.y,out.orientation.z,out.orientation.w);
            const auto gravity=tf2::quatRotate(q, tf2::Vector3(-out.linear_acceleration.x,
                -out.linear_acceleration.y,-out.linear_acceleration.z));
            check(std::hypot(gravity.x(),gravity.y())<0.02);
            check(std::abs(gravity.z()-9.80665)<0.03);
            const auto legacy=rcs_imu::convert(data,settings,mount,true);
            tf2::Quaternion legacy_q(legacy.orientation.x,legacy.orientation.y,legacy.orientation.z,legacy.orientation.w);
            const auto wrong=tf2::quatRotate(legacy_q,tf2::Vector3(-legacy.linear_acceleration.x,
                -legacy.linear_acceleration.y,-legacy.linear_acceleration.z));
            check(std::hypot(wrong.x(),wrong.y())>0.2);
        }
    }
    data.reset();
    data.quaternion.data[0]=0;
    bool rejected=false;
    try { rcs_imu::convert(data,settings,mount); } catch (const std::runtime_error &) { rejected=true; }
    check(rejected);
    data.quaternion.data[0]=1;
    data.accCalibrated.data[0]=std::numeric_limits<float>::quiet_NaN();
    rejected=false;
    try { rcs_imu::convert(data,settings,mount); } catch (const std::runtime_error &) { rejected=true; }
    check(rejected);
    std::cout << "PASS: units, mount rotation, quaternion, recorded gravity, missing fields, invalid data\n";
}
