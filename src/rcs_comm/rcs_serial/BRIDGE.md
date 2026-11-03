# 串口桥接

业务消息格式与注册表由 `rcs_msgs` 管理，见 [业务协议 v1](../rcs_msgs/BUSINESS.md)。本包不定义动作参数或比赛逻辑。

- `/action/command` (`rcs_msgs/ActionCommand`)：动作请求、取消。
- `/action/feedback` (`rcs_msgs/ActionFeedback`)：下位机动作状态。
- `/cmd/chassis` (`rcs_msgs/ChassisCmd`)：保留原底盘发送格式。
- `/vision/object` (`rcs_msgs/VisionData`)：保留视觉发送格式，未知目标类型编码为 0。

ROS 回调使用默认单线程执行器，编解码与反馈发布在执行器线程完成。`SerialTransport` 独立线程读取串口，`RdlcSerialLink` 将解出的帧放进队列，桥接定时消费；单次最多处理 64 帧。发送异常会记录日志，由动作超时向任务报告失败。

```bash
ros2 run rcs_serial serial_bridge --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p escape_enabled:=true
```

转义必须两端同时配置。桥接仅接受配置的远端源地址和本地目标地址。

安装并 source 相关包后，可用伪串口模拟 MCU 验证完整闭环（需要 rclpy、pyserial，无需硬件）：

```bash
python3 src/rcs_comm/rcs_serial/test/integration_action_link.py
python3 -m unittest discover -s src/rcs_decision/rcs_task/test -v
```
