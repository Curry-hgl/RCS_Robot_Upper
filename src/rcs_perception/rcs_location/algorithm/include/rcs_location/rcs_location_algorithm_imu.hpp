#pragma once
#include <array>
#include <cstddef>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace rcs_location {
struct ImuOptions {
  double calibration_seconds{2.0};
  int calibration_samples{100};
  double stationary_gyro{0.08};
  double stationary_acceleration{0.3};
  double max_dt{0.1};
  double acceleration_sign{-1.0};
  double gravity_magnitude{9.80665};
  double acceleration_noise{0.2};
  double bias_noise{0.02};
  double yaw_noise{0.02};
  bool deadband_enabled{true};
  double deadband_sigma{5.0};
  bool zero_velocity_enabled{true};
  double zero_velocity_seconds{1.0};
  double zero_velocity_gyro{0.03};
  double motion_confirm_seconds{0.15};
  tf2::Vector3 acceleration_deadband{0.1,0.1,0};
  tf2::Vector3 imu_position{0, 0, 0};
};
enum class ImuResult { Invalid, Calibrating, Initialized, Updated, Gap };
struct ImuEstimate {
  double x{0}, y{0}, yaw{0}, vx{0}, vy{0}, wz{0};
  double position_variance{0}, velocity_variance{0};
};

// ROS 无关：输入姿态为 IMU→世界，安装旋转由驱动统一处理。
class ImuAlgorithm {
 public:
  explicit ImuAlgorithm(const ImuOptions &options);
  void reset();
  ImuResult update(double stamp, tf2::Quaternion orientation,
                   const tf2::Vector3 &acceleration, const tf2::Vector3 &gyro);
  const ImuEstimate &estimate() const { return estimate_; }
  bool ready() const { return ready_; }
  const tf2::Vector3 &deadband() const { return deadband_; }
 private:
  void clear_calibration();
  ImuOptions options_;
  ImuEstimate estimate_;
  bool ready_{false}, have_stamp_{false}, have_acceleration_{false};
  double last_stamp_{0}, calibration_start_{0}, elapsed_{0}, initial_yaw_{0};
  std::size_t samples_{0};
  tf2::Vector3 gravity_{0,0,0}, gravity_sum_{0,0,0}, gyro_sum_{0,0,0};
  tf2::Vector3 acceleration_bias_{0,0,0};
  tf2::Vector3 first_acceleration_{0,0,0}, gyro_bias_{0,0,0};
  tf2::Vector3 last_acceleration_{0,0,0}, initial_lever_{0,0,0};
  tf2::Vector3 position_{0,0,0}, velocity_{0,0,0};
  struct CalibrationSample { tf2::Quaternion orientation; tf2::Vector3 acceleration; };
  std::vector<CalibrationSample> calibration_data_;
  tf2::Vector3 deadband_{0,0,0};
  double quiet_seconds_{0}, motion_seconds_{0};
  bool zero_velocity_active_{false};
  double pp_{0}, pv_{0}, vv_{0};
};
}  // namespace rcs_location
