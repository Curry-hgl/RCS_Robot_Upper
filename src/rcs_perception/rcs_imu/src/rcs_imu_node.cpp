#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "LpmsIG1I.h"
#include "rcs_imu/rcs_imu_conversion.hpp"

class RcsImuNode : public rclcpp::Node {
    struct Release { void operator()(IG1I *p) const { if (p) p->release(); } };
    using Trigger = std_srvs::srv::Trigger;
    std::unique_ptr<IG1I, Release> sensor_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_, gyro_pub_;
    std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
    std::string frame_;
    tf2::Quaternion mount_;
    bool quaternion_inverse_{false};
    bool reconnect_, degrees_, connected_ = false, reported_ = false;
    int last_status_ = -1;
    double timeout_;
    std::chrono::steady_clock::time_point waiting_, calibration_until_{};

    void command_service(const std::string &name, std::function<void()> command) {
        services_.push_back(create_service<Trigger>("~/"+name,
            [this, command, name](const std::shared_ptr<Trigger::Request>,
                                 std::shared_ptr<Trigger::Response> response) {
                if (sensor_->getStatus() != STATUS_CONNECTED) {
                    response->message = "IMU not connected"; return;
                }
                const auto now = std::chrono::steady_clock::now();
                if (now < calibration_until_) {
                    response->message = "Gyroscope calibration in progress"; return;
                }
                command();
                response->success = true;
                response->message = "Command queued; completion is not confirmed";
                if (name == "calibrate_gyroscope") {
                    calibration_until_ = now + std::chrono::seconds(4);
                    response->message = "Calibration queued; keep IMU stationary for at least 4 seconds";
                }
            }));
    }

    void debug(const sensor_msgs::msg::Imu &message) {
        const double scale = degrees_ ? 180.0/std::acos(-1.0) : 1.0;
        geometry_msgs::msg::Vector3Stamped out;
        out.header = message.header;
        if (message.orientation_covariance[0] != -1) {
            const auto &q = message.orientation;
            double r, p, y;
            tf2::Matrix3x3(tf2::Quaternion(q.x,q.y,q.z,q.w)).getRPY(r,p,y);
            out.vector.x=r*scale; out.vector.y=p*scale; out.vector.z=y*scale;
            rpy_pub_->publish(out);
        }
        if (message.angular_velocity_covariance[0] != -1) {
            out.vector=message.angular_velocity;
            out.vector.x*=scale; out.vector.y*=scale; out.vector.z*=scale;
            gyro_pub_->publish(out);
        }
    }

    void tick() {
        const int status = sensor_->getStatus();
        const bool connected = status == STATUS_CONNECTED;
        const auto now = std::chrono::steady_clock::now();
        if (status != last_status_) {
            RCLCPP_INFO(get_logger(), "IMU connection state: %d", status);
            last_status_=status;
        }
        if (connected != connected_) {
            connected_=connected;
            if (connected) {
                RCLCPP_INFO(get_logger(), "IMU connected");
                reported_=false;
            } else { waiting_=now; }
            std_msgs::msg::Bool state; state.data=connected;
            connected_pub_->publish(state);
        }
        if (!connected) {
            if (!reported_ && std::chrono::duration<double>(now-waiting_).count() > timeout_) {
                RCLCPP_ERROR(get_logger(), "IMU connection timeout; check UART, power and baudrate");
                reported_=true;
            }
            if (!reconnect_ && (status == STATUS_CONNECTION_ERROR || reported_))
                throw std::runtime_error("IMU connection failed (autoreconnect disabled)");
            return;
        }
        IG1SettingsI settings;
        sensor_->getSettings(settings);
        IG1ImuDataI latest;
        bool received = false;
        // 丢弃积压旧帧，仅发布最新采样，避免延迟持续增长。
        for (int i=0; i<256 && sensor_->getImuData(latest); ++i) received=true;
        if (!received) return;
        try {
            auto message=rcs_imu::convert(latest, settings, mount_, quaternion_inverse_);
            message.header.stamp=this->now();
            message.header.frame_id=frame_;
            publisher_->publish(message);
            if (rpy_pub_) debug(message);
        } catch (const std::exception &error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
        }
    }

public:
    RcsImuNode() : Node("rcs_imu") {
        const auto port=declare_parameter<std::string>("port", "/dev/ttyS1");
        const int baud=declare_parameter<int>("baudrate",115200);
        frame_=declare_parameter<std::string>("frame_id","imu_link");
        const double rate=declare_parameter<double>("rate",200.0);
        timeout_=declare_parameter<double>("connection_timeout",15.0);
        reconnect_=declare_parameter<bool>("autoreconnect",true);
        const bool debug_enabled=declare_parameter<bool>("debug_enabled",false);
        degrees_=declare_parameter<bool>("debug_degrees",true);
        quaternion_inverse_=declare_parameter<bool>("quaternion_inverse",false);
        const auto rpy=declare_parameter<std::vector<double>>("mount_rpy",{0.0,0.0,0.0});
        if (port.empty() || baud<=0 || frame_.empty() || !std::isfinite(rate) || rate<=0 || rate>10000 ||
            !std::isfinite(timeout_) || timeout_<=0 || rpy.size()!=3)
            throw std::invalid_argument("Invalid IMU configuration");
        for (auto angle:rpy) if (!std::isfinite(angle)) throw std::invalid_argument("Invalid mount_rpy");
        mount_.setRPY(rpy[0],rpy[1],rpy[2]);
        publisher_=create_publisher<sensor_msgs::msg::Imu>("~/data",rclcpp::SensorDataQoS());
        connected_pub_=create_publisher<std_msgs::msg::Bool>("~/connected",rclcpp::QoS(1).transient_local());
        std_msgs::msg::Bool state; state.data=false; connected_pub_->publish(state);
        if (debug_enabled) {
            const std::string suffix=degrees_ ? "_deg" : "_rad";
            rpy_pub_=create_publisher<geometry_msgs::msg::Vector3Stamped>("~/rpy"+suffix,10);
            gyro_pub_=create_publisher<geometry_msgs::msg::Vector3Stamped>("~/angular_velocity"+suffix,10);
        }
        sensor_.reset(IG1Factory());
        if (!sensor_) throw std::runtime_error("Cannot create LPMS driver");
        sensor_->setVerbose(VERBOSE_NONE);
        sensor_->setAutoReconnectStatus(reconnect_);
        sensor_->connect(port,baud);
        waiting_=std::chrono::steady_clock::now();
        command_service("calibrate_gyroscope",[this]{sensor_->commandStartGyroCalibration();});
        command_service("reset_heading",[this]{sensor_->commandSetOffsetMode(LPMS_OFFSET_MODE_HEADING);});
        timer_=create_wall_timer(std::chrono::duration<double>(1.0/rate),[this]{tick();});
        RCLCPP_INFO(get_logger(), "IMU %s @ %d, frame=%s",port.c_str(),baud,frame_.c_str());
    }
    ~RcsImuNode() override { timer_.reset(); sensor_.reset(); }
};

int main(int argc,char **argv) {
    rclcpp::init(argc,argv);
    int result=0;
    try { rclcpp::spin(std::make_shared<RcsImuNode>()); }
    catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("rcs_imu"),"%s",error.what()); result=1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return result;
}
