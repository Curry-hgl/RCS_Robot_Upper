"""工创功能接口：只负责发布操作，不决定二维码阶段或何时切换模式。"""
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcs_msgs.msg import GcsDetections, GcsTaskCode, GcsMode


class GcsActions:
    def __init__(self, node):
        self.blocks = node.create_publisher(GcsDetections, "/gcs/blocks", 10)
        self.rings = node.create_publisher(GcsDetections, "/gcs/rings", 10)
        self.codes = node.create_publisher(GcsTaskCode, "/gcs/task_code/send", 10)
        # 模式属于持久状态，晚启动的视觉节点也应得到最近一次选择。
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.modes = node.create_publisher(GcsMode, "/gcs/detection_mode", qos)

    def set_mode(self, mode):
        self.modes.publish(GcsMode(mode=mode))

    def send_code(self, raw):
        self.codes.publish(GcsTaskCode(raw=raw))

    def send_detections(self, msg):
        (self.blocks if msg.mode == 0 else self.rings).publish(msg)
