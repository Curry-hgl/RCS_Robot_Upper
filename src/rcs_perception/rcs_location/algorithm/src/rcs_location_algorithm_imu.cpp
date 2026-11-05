#include "rcs_location/rcs_location_algorithm_imu.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rcs_location {
namespace {
bool finite(const tf2::Vector3 &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
double yaw(const tf2::Quaternion &q) {
  double r, p, y; tf2::Matrix3x3(q).getRPY(r, p, y); return y;
}
}
ImuAlgorithm::ImuAlgorithm(const ImuOptions &o) : options_(o) {
  for (double value : {o.gravity_magnitude, o.calibration_seconds, o.stationary_gyro,
      o.stationary_acceleration, o.max_dt, o.acceleration_noise, o.bias_noise, o.yaw_noise, o.deadband_sigma, o.zero_velocity_seconds, o.zero_velocity_gyro}) {
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("IMU thresholds must be finite and positive");
  }
  if (!std::isfinite(o.motion_confirm_seconds) || o.motion_confirm_seconds < 0)
    throw std::invalid_argument("motion_confirm_seconds must be finite and nonnegative");
  if (!finite(o.acceleration_deadband) || o.acceleration_deadband.x() < 0 || o.acceleration_deadband.y() < 0)
    throw std::invalid_argument("acceleration_deadband must be finite and nonnegative");
  deadband_ = options_.acceleration_deadband;
  if (o.calibration_samples < 2 || (o.acceleration_sign != 1 && o.acceleration_sign != -1) || !finite(o.imu_position))
    throw std::invalid_argument("invalid IMU calibration, sign or position");
}
void ImuAlgorithm::clear_calibration() {
  samples_ = 0; gravity_sum_.setZero(); gyro_sum_.setZero(); calibration_data_.clear();
}
void ImuAlgorithm::reset() {
  ready_ = have_stamp_ = have_acceleration_ = false;
  estimate_ = {}; position_.setZero(); velocity_.setZero(); elapsed_ = pp_ = pv_ = vv_ = 0;
  deadband_ = options_.acceleration_deadband;
  quiet_seconds_ = motion_seconds_ = 0; zero_velocity_active_ = false;
  clear_calibration();
}
ImuResult ImuAlgorithm::update(double stamp, tf2::Quaternion q,
    const tf2::Vector3 &acceleration, const tf2::Vector3 &gyro) {
  if (!std::isfinite(stamp) || !finite(acceleration) || !finite(gyro) ||
      !std::isfinite(q.length2()) || q.length2() < 1e-12) return ImuResult::Invalid;
  q.normalize();
  if (have_stamp_ && stamp <= last_stamp_) return ImuResult::Invalid;
  const double dt = have_stamp_ ? stamp - last_stamp_ : 0;
  last_stamp_ = stamp; have_stamp_ = true;
  const auto world_acceleration = tf2::quatRotate(q, acceleration * options_.acceleration_sign);
  if (!ready_) {
    if (dt > options_.max_dt) clear_calibration();
    if (gyro.length() > options_.stationary_gyro || acceleration.length() < 8 || acceleration.length() > 12) {
      clear_calibration(); return ImuResult::Calibrating;
    }
    if (samples_ && (world_acceleration - first_acceleration_).length() > options_.stationary_acceleration)
      clear_calibration();
    if (!samples_) { calibration_start_ = stamp; first_acceleration_ = world_acceleration; }
    // 固定世界重力方向，用静止样本估计机体系残余偏置。
    const tf2::Vector3 reference(0, 0, options_.gravity_magnitude);
    gravity_sum_ += acceleration * options_.acceleration_sign - tf2::quatRotate(q.inverse(), reference); gyro_sum_ += gyro; ++samples_;
    if (options_.deadband_enabled) calibration_data_.push_back({q, acceleration});
    if (samples_ < static_cast<std::size_t>(options_.calibration_samples) ||
        stamp - calibration_start_ < options_.calibration_seconds) return ImuResult::Calibrating;
    gravity_ = tf2::Vector3(0, 0, options_.gravity_magnitude);
    acceleration_bias_ = gravity_sum_ / static_cast<double>(samples_);
    gyro_bias_ = gyro_sum_ / static_cast<double>(samples_);
    initial_yaw_ = yaw(q);
    tf2::Quaternion heading; heading.setRPY(0, 0, -initial_yaw_);
    initial_lever_ = tf2::quatRotate(heading * q, options_.imu_position);
    if (options_.deadband_enabled) {
      tf2::Vector3 mean(0,0,0), m2(0,0,0);
      std::size_t count = 0;
      for (const auto &sample : calibration_data_) {
        const auto corrected = tf2::quatRotate(sample.orientation,
            sample.acceleration * options_.acceleration_sign - acceleration_bias_);
        const auto linear = tf2::quatRotate(heading, corrected - gravity_);
        const auto delta = linear - mean;
        mean += delta / static_cast<double>(++count);
        const auto remaining = linear - mean;
        for (int axis=0;axis<2;++axis) m2[axis] += delta[axis] * remaining[axis];
      }
      for (int axis=0;axis<2;++axis) {
        const double sigma = std::sqrt(std::max(0.0, m2[axis] / static_cast<double>(count - 1)));
        deadband_[axis] = std::max(options_.acceleration_deadband[axis], options_.deadband_sigma * sigma);
      }
      calibration_data_.clear();
    }
    // 初始化已完成静止确认，直接进入清零状态，避免启动后的短脉冲。
    zero_velocity_active_ = options_.deadband_enabled && options_.zero_velocity_enabled;
    ready_ = true;
    return ImuResult::Initialized;
  }
  tf2::Quaternion heading; heading.setRPY(0, 0, -initial_yaw_);
  const auto body_to_odom = heading * q;
  const auto corrected_world = tf2::quatRotate(q, acceleration * options_.acceleration_sign - acceleration_bias_);
  auto linear = tf2::quatRotate(heading, corrected_world - gravity_);
  const bool small_acceleration = std::abs(linear.x()) <= deadband_.x() && std::abs(linear.y()) <= deadband_.y();
  if (options_.deadband_enabled) {
    for (int axis=0;axis<2;++axis) {
      const double magnitude = std::max(0.0, std::abs(linear[axis]) - deadband_[axis]);
      linear[axis] = std::copysign(magnitude, linear[axis]);
    }
  }
  const auto lever = tf2::quatRotate(body_to_odom, options_.imu_position);
  const auto body_gyro = gyro - gyro_bias_;
  const auto world_gyro = tf2::quatRotate(body_to_odom, body_gyro);
  estimate_.yaw = yaw(body_to_odom); estimate_.wz = body_gyro.z();
  if (dt > options_.max_dt) {
    // 缺帧期间位移不可恢复；保留坐标，速度归零并抬高不确定度。
    have_acceleration_ = false; quiet_seconds_ = motion_seconds_ = 0; zero_velocity_active_ = false;
    velocity_.setZero(); estimate_.vx = estimate_.vy = 0;
    pp_ += 1; vv_ += 1;
    elapsed_ += dt;
    estimate_.position_variance = pp_ + std::pow(elapsed_, 4) * options_.bias_noise * options_.bias_noise / 4;
    estimate_.velocity_variance = vv_ + elapsed_ * elapsed_ * options_.bias_noise * options_.bias_noise;
    return ImuResult::Gap;
  }
  const bool quiet = options_.deadband_enabled && options_.zero_velocity_enabled &&
      small_acceleration && body_gyro.length() <= options_.zero_velocity_gyro;
  if (quiet) {
    quiet_seconds_ += dt; motion_seconds_ = 0;
    if (quiet_seconds_ >= options_.zero_velocity_seconds) zero_velocity_active_ = true;
  } else {
    quiet_seconds_ = 0;
    if (zero_velocity_active_) {
      motion_seconds_ += dt;
      if (motion_seconds_ >= options_.motion_confirm_seconds) zero_velocity_active_ = false;
    }
  }
  const auto a = have_acceleration_ ? (last_acceleration_ + linear) / 2 : linear;
  // 内部积分 IMU 原点，发布时减去安装杆臂及其旋转速度。
  // 仅为低动态启发式；实际静止约束后续由轮速等外部信息提供。
  if (zero_velocity_active_) velocity_.setZero();
  else {
    position_ += velocity_ * dt + a * (0.5 * dt * dt);
    velocity_ += a * dt;
  }
  const auto body_position = position_ - lever + initial_lever_;
  const auto body_velocity = velocity_ - world_gyro.cross(lever);
  estimate_.x = body_position.x(); estimate_.y = body_position.y();
  estimate_.vx = body_velocity.x(); estimate_.vy = body_velocity.y();
  last_acceleration_ = linear; have_acceleration_ = true;
  const double noise = options_.acceleration_noise * options_.acceleration_noise;
  pp_ += 2 * dt * pv_ + dt * dt * vv_ + 0.25 * std::pow(dt, 4) * noise;
  pv_ += dt * vv_ + 0.5 * std::pow(dt, 3) * noise;
  vv_ += dt * dt * noise; elapsed_ += dt;
  estimate_.position_variance = pp_ + std::pow(elapsed_, 4) * options_.bias_noise * options_.bias_noise / 4;
  estimate_.velocity_variance = vv_ + elapsed_ * elapsed_ * options_.bias_noise * options_.bias_noise;
  return ImuResult::Updated;
}
}  // namespace rcs_location
