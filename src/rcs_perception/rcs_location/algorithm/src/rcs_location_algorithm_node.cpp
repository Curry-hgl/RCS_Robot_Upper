#include "rcs_location/rcs_location_algorithm_imu.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include "rcs_location/msg/rcs_location_state.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace rcs_location {
class AlgorithmNode : public rclcpp::Node {
 public:
  AlgorithmNode() : Node("rcs_location_algorithm") {
    ImuOptions options;
    options.calibration_seconds = declare_parameter("calibration_seconds", 2.0);
    options.calibration_samples = declare_parameter("calibration_samples", 100);
    options.stationary_gyro = declare_parameter("stationary_gyro", 0.08);
    options.stationary_acceleration = declare_parameter("stationary_acceleration", 0.3);
    options.max_dt = declare_parameter("max_dt", 0.1);
    options.gravity_magnitude = declare_parameter("gravity_magnitude", 9.80665);
    options.acceleration_sign = declare_parameter("acceleration_sign", -1.0);
    options.acceleration_noise = declare_parameter("acceleration_noise", 0.2);
    options.bias_noise = declare_parameter("bias_noise", 0.02);
    options.yaw_noise = declare_parameter("yaw_noise", 0.02);
    options.deadband_enabled = declare_parameter("deadband_enabled", true);
    options.zero_velocity_enabled = declare_parameter("zero_velocity_enabled", true);
    options.zero_velocity_seconds = declare_parameter("zero_velocity_seconds", 1.0);
    options.motion_confirm_seconds = declare_parameter("motion_confirm_seconds", 0.15);
    options.zero_velocity_gyro = declare_parameter("zero_velocity_gyro", 0.03);
    options.deadband_sigma = declare_parameter("deadband_sigma", 5.0);
    const auto deadband = declare_parameter("acceleration_deadband", std::vector<double>{0.1,0.1});
    if (deadband.size() != 2) throw std::invalid_argument("acceleration_deadband needs x,y");
    options.acceleration_deadband = tf2::Vector3(deadband[0],deadband[1],0);
    yaw_variance_ = options.yaw_noise * options.yaw_noise;
    const auto gyro_noise = declare_parameter("gyro_noise", 0.01);
    if (!std::isfinite(gyro_noise) || gyro_noise <= 0) throw std::invalid_argument("gyro_noise must be positive");
    gyro_variance_ = gyro_noise * gyro_noise;
    const auto position = declare_parameter("imu_position", std::vector<double>{0,0,0});
    if (position.size() != 3) throw std::invalid_argument("imu_position needs x,y,z");
    options.imu_position = tf2::Vector3(position[0], position[1], position[2]);
    algorithm_ = std::make_unique<ImuAlgorithm>(options);
    frame_ = declare_parameter("odom_frame", "odom");
    body_ = declare_parameter("body_frame", "base_link");
    source_frame_ = declare_parameter("imu_frame", "imu_link");
    const auto topic = declare_parameter("imu_topic", "/rcs_imu/data");
    const auto output = declare_parameter("odom_topic", "/rcs_location/algorithm/imu/odom");
    const auto state_topic = declare_parameter("state_topic", "/rcs_location/algorithm/imu/state");
    length_unit_ = declare_parameter("length_unit", "cm");
    angle_unit_ = declare_parameter("angle_unit", "deg");
    if (length_unit_ == "m") length_scale_ = 1;
    else if (length_unit_ == "cm") length_scale_ = 100;
    else if (length_unit_ == "mm") length_scale_ = 1000;
    else throw std::invalid_argument("length_unit must be m, cm or mm");
    if (angle_unit_ == "rad") angle_scale_ = 1;
    else if (angle_unit_ == "deg") angle_scale_ = 180 / std::acos(-1.0);
    else throw std::invalid_argument("angle_unit must be rad or deg");
    const auto precision = declare_parameter("state_precision", 2);
    if (precision < 0 || precision > 6) throw std::invalid_argument("state_precision must be 0..6");
    rounding_scale_ = std::pow(10.0, precision);
    state_ = create_publisher<rcs_location::msg::RcsLocationState>(state_topic, 10);
    timeout_ = declare_parameter("input_timeout", 1.0);
    if (frame_.empty() || body_.empty() || source_frame_.empty() || frame_ == body_ ||
        !std::isfinite(timeout_) || timeout_ <= 0) throw std::invalid_argument("invalid frame or timeout");
    if (declare_parameter("publish_tf", false)) broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output, 10);
    ready_ = create_publisher<std_msgs::msg::Bool>("~/ready", rclcpp::QoS(1).transient_local());
    set_ready(false);
    reset_ = create_service<std_srvs::srv::Trigger>("~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      algorithm_->reset(); set_ready(false);
      response->success = true; response->message = "Origin cleared; keep stationary for initialization";
    });
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { consume(*message); });
    last_input_ = std::chrono::steady_clock::now();
    watchdog_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_input_).count() > timeout_) {
        set_ready(false);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No valid IMU input; check driver and topic");
      }
    });
    RCLCPP_INFO(get_logger(), "IMU %s -> %s; keep stationary for %.1f s", topic.c_str(), output.c_str(), options.calibration_seconds);
  }
 private:
  void set_ready(bool value) { std_msgs::msg::Bool msg; msg.data = value; ready_->publish(msg); }
  void consume(const sensor_msgs::msg::Imu &msg) {
    if (msg.header.frame_id != source_frame_ || msg.orientation_covariance[0] < 0 ||
        msg.linear_acceleration_covariance[0] < 0 || msg.angular_velocity_covariance[0] < 0) {
      set_ready(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU frame mismatch or missing orientation/acceleration/gyro");
      return;
    }
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const auto result = algorithm_->update(stamp,
        tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
        tf2::Vector3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
        tf2::Vector3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
    if (result == ImuResult::Invalid) {
      set_ready(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid IMU sample or non-increasing timestamp"); return;
    }
    last_input_ = std::chrono::steady_clock::now();
    set_ready(algorithm_->ready() && result != ImuResult::Gap);
    if (result == ImuResult::Calibrating) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Initializing IMU; keep stationary"); return;
    }
    if (result == ImuResult::Initialized) {
      RCLCPP_INFO(get_logger(), "IMU origin initialized; acceleration deadband x=%.4f y=%.4f m/s^2",
                  algorithm_->deadband().x(), algorithm_->deadband().y());
    }
    if (result == ImuResult::Gap) {
      RCLCPP_WARN(get_logger(), "IMU data gap; velocity cleared, position during gap is unknown"); return;
    }
    const auto &value = algorithm_->estimate();
    nav_msgs::msg::Odometry out;
    out.header = msg.header; out.header.frame_id = frame_; out.child_frame_id = body_;
    out.pose.pose.position.x = value.x; out.pose.pose.position.y = value.y;
    out.pose.pose.orientation.z = std::sin(value.yaw / 2);
    out.pose.pose.orientation.w = std::cos(value.yaw / 2);
    // Odometry 的 twist 必须在 child_frame_id 下表达。
    out.twist.twist.linear.x = std::cos(value.yaw) * value.vx + std::sin(value.yaw) * value.vy;
    out.twist.twist.linear.y = -std::sin(value.yaw) * value.vx + std::cos(value.yaw) * value.vy;
    out.twist.twist.angular.z = value.wz;
    out.pose.covariance[0] = out.pose.covariance[7] = std::max(value.position_variance, 1e-6);
    out.twist.covariance[0] = out.twist.covariance[7] = std::max(value.velocity_variance, 1e-6);
    out.pose.covariance[35] = yaw_variance_;
    out.twist.covariance[35] = gyro_variance_;
    for (int index : {14,21,28}) { out.pose.covariance[index] = 1e6; out.twist.covariance[index] = 1e6; }
    publisher_->publish(out);
    // 带单位的查看接口；Odometry 和 TF 始终使用 SI 单位。
    const auto rounded = [this](double number) { return std::round(number * rounding_scale_) / rounding_scale_; };
    rcs_location::msg::RcsLocationState state;
    state.length_unit = length_unit_; state.angle_unit = angle_unit_;
    state.x = rounded(value.x * length_scale_); state.y = rounded(value.y * length_scale_);
    state.yaw = rounded(value.yaw * angle_scale_);
    state.vx = rounded(out.twist.twist.linear.x * length_scale_);
    state.vy = rounded(out.twist.twist.linear.y * length_scale_);
    state.wz = rounded(value.wz * angle_scale_);
    state_->publish(state);
    if (broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = out.header; tf.child_frame_id = body_;
      tf.transform.translation.x = value.x; tf.transform.translation.y = value.y;
      tf.transform.rotation = out.pose.pose.orientation; broadcaster_->sendTransform(tf);
    }
  }
  std::unique_ptr<ImuAlgorithm> algorithm_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  rclcpp::Publisher<rcs_location::msg::RcsLocationState>::SharedPtr state_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::chrono::steady_clock::time_point last_input_;
  std::string frame_, body_, source_frame_;
  std::string length_unit_, angle_unit_;
  double length_scale_, angle_scale_, rounding_scale_;
  double yaw_variance_, gyro_variance_, timeout_;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); int result = 0;
  try { rclcpp::spin(std::make_shared<rcs_location::AlgorithmNode>()); }
  catch (const std::exception &exc) { RCLCPP_ERROR(rclcpp::get_logger("rcs_location_algorithm"), "%s", exc.what()); result = 1; }
  rclcpp::shutdown(); return result;
}
