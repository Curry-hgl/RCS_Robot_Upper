"""工创任务 ROS 适配；单线程回调驱动任务，视觉和串口独立运行。"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcs_action.gcs import GcsActions
from rcs_msgs.msg import GcsTaskCode, GcsMode, GcsDetections, GcsGripperEvent, GcsStatus
from .gcs import GcsTask


class GcsTaskNode(Node):
    def __init__(self):
        super().__init__("gcs_task")
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.task = GcsTask(GcsActions(self), time.monotonic,
            qr_enabled=param("qr_enabled", True),
            send_interval=param("send_interval", 0.5),
            broadcast_duration=param("qr_send_duration", 5.0),
            initial_mode=param("initial_mode", 0))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(GcsStatus, "/gcs/status", qos)
        self.subs = [
            self.create_subscription(GcsTaskCode, "/gcs/qr_candidate", lambda m: self.task.on_qr(m.raw), 10),
            self.create_subscription(GcsMode, "/gcs/mode_command", lambda m: self.task.on_mode(m.mode), 10),
            self.create_subscription(GcsDetections, "/gcs/detections", self.task.on_detections, 10),
            self.create_subscription(GcsGripperEvent, "/gcs/gripper_event", self.task.on_gripper, 10),
        ]
        self.timer = self.create_timer(0.05, self._tick)

    def _tick(self):
        self.task.tick()
        task = self.task
        self.status.publish(GcsStatus(task_code=task.code, pick_count=task.pick_count,
            place_count=task.place_count, mode=task.mode, phase=task.machine.state.name))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GcsTaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
