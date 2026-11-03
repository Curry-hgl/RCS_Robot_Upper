"""ROS 与串口桥接：订阅业务消息，经 msgs 编码交给 RDLC；回帧解码后发布。

串口读取线程仅收字节，业务处理在 ROS 执行器中完成；不包含赛题决策。
"""
from queue import Empty

import rclpy
from rclpy.node import Node
from rcs_msgs.msg import ActionCommand, ActionFeedback, ChassisCmd, VisionData
from rcs_msgs.business import (
    ACTION_COMMAND, ACTION_FEEDBACK, CHASSIS, VISION, default_registry,
)
from rcs_msgs.msg import GcsDetections, GcsTaskCode, GcsMode, GcsGripperEvent
from rcs_msgs.business.gcs import BLOCKS, RINGS, TASK_CODE, MODE, GRIPPER
from .config import SerialConfig
from .rdlc_link import RdlcSerialLink


class SerialBridge(Node):
    def __init__(self, config=None):
        super().__init__("serial_bridge")
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        baud = self.declare_parameter("baudrate", 115200).value
        escape = self.declare_parameter("escape_enabled", False).value
        self.link = RdlcSerialLink(config or SerialConfig(
            port=port, baudrate=baud, escape_enabled=escape))
        self.registry = default_registry()
        self.feedback_pub = self.create_publisher(ActionFeedback, "/action/feedback", 10)
        self.mode_pub = self.create_publisher(GcsMode, "/gcs/mode_command", 10)
        self.gripper_pub = self.create_publisher(GcsGripperEvent, "/gcs/gripper_event", 10)
        self._route_subscriptions = []
        for topic, kind, identifier in (
            ("/gcs/blocks", GcsDetections, BLOCKS),
            ("/gcs/rings", GcsDetections, RINGS),
            ("/gcs/task_code/send", GcsTaskCode, TASK_CODE),
            ("/action/command", ActionCommand, ACTION_COMMAND),
            ("/cmd/chassis", ChassisCmd, CHASSIS),
            ("/vision/object", VisionData, VISION),
        ):
            self._route_subscriptions.append(self.create_subscription(
                kind, topic, lambda msg, key=identifier: self._send(key, msg), 10))
        # SerialTransport owns the reader thread. ROS publishing and all business
        # processing stay in the single executor thread.
        self.timer = self.create_timer(0.01, self._receive)

    def _send(self, identifier, msg):
        try:
            self.link.send(self.registry.encode(identifier, msg))
        except Exception as exc:
            self.get_logger().error(f"Command send failed: {exc}")

    def _receive(self):
        for _ in range(64):
            try:
                frame = self.link.receive(timeout=0)
            except Empty:
                break
            if (frame.dst != self.link.config.local_address
                    or frame.src != self.link.config.remote_address):
                continue
            try:
                if not frame.payload:
                    continue
                identifier = frame.payload[0]
                if identifier == MODE:
                    mode = self.registry.decode(frame.payload)
                    self.mode_pub.publish(GcsMode(mode=mode.mode))
                    continue
                if identifier == GRIPPER:
                    event = self.registry.decode(frame.payload)
                    self.gripper_pub.publish(GcsGripperEvent(
                        action=event.action, pick=event.pick, place=event.place))
                    continue
                if identifier != ACTION_FEEDBACK:
                    continue
                feedback = self.registry.decode(frame.payload)
                msg = ActionFeedback()
                msg.request_id = feedback.request_id
                msg.action_code = feedback.action_code
                msg.status = feedback.status
                msg.error_code = feedback.error_code
                msg.data = list(feedback.data)
                self.feedback_pub.publish(msg)
            except (ValueError, TypeError) as exc:
                self.get_logger().warning(f"Invalid business feedback: {exc}")

    def open(self):
        self.link.open()

    def close(self):
        self.timer.cancel()
        self.link.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        node.open()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
