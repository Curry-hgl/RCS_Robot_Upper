#pragma once
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "SensorDataI.h"
#include "LpmsIG1Registers.h"

namespace rcs_imu {
inline sensor_msgs::msg::Imu convert(const IG1ImuDataI &data, const IG1SettingsI &settings,
                                    const tf2::Quaternion &mount) {
    sensor_msgs::msg::Imu out;
    const auto flags = settings.transmitDataConfig;
    out.orientation_covariance[0] = -1;
    out.angular_velocity_covariance[0] = -1;
    out.linear_acceleration_covariance[0] = -1;
    if (flags & TDR_QUAT_OUTPUT_ENABLED) {
        // LPMS 四元数为参考系到传感器系；ROS 使用相反方向。
        tf2::Quaternion q(-data.quaternion.data[1], -data.quaternion.data[2],
                          -data.quaternion.data[3], data.quaternion.data[0]);
        if (!std::isfinite(q.length2()) || q.length2() < 1e-12)
            throw std::runtime_error("Invalid IMU quaternion");
        q.normalize();
        q *= mount.inverse();
        out.orientation.x=q.x(); out.orientation.y=q.y();
        out.orientation.z=q.z(); out.orientation.w=q.w();
        out.orientation_covariance.fill(0.0); // 协方差未知
    }
    const tf2::Matrix3x3 rotation(mount);
    auto assign = [](auto &target, const tf2::Vector3 &v) {
        if (!std::isfinite(v.x()) || !std::isfinite(v.y()) || !std::isfinite(v.z()))
            throw std::runtime_error("Non-finite IMU vector");
        target.x=v.x(); target.y=v.y(); target.z=v.z();
    };
    if (flags & TDR_GYR1_ALIGN_CALIBRATED_OUTPUT_ENABLED) {
        const double scale = settings.useRadianOutput ? 1.0 : std::acos(-1.0)/180.0;
        assign(out.angular_velocity, rotation * tf2::Vector3(
            data.gyroIIAlignmentCalibrated.data[0]*scale,
            data.gyroIIAlignmentCalibrated.data[1]*scale,
            data.gyroIIAlignmentCalibrated.data[2]*scale));
        out.angular_velocity_covariance.fill(0.0);
    }
    if (flags & TDR_ACC_CALIBRATED_OUTPUT_ENABLED) {
        // 沿用 BE 例程的加速度符号，将 g 转为 m/s²。
        assign(out.linear_acceleration, rotation * tf2::Vector3(
            -data.accCalibrated.data[0]*9.80665,
            -data.accCalibrated.data[1]*9.80665,
            -data.accCalibrated.data[2]*9.80665));
        out.linear_acceleration_covariance.fill(0.0);
    }
    return out;
}
}
