"""非阻塞功能接口：发布业务指令，不决定比赛流程。"""
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcs_msgs.msg import RcsDetections, RcsTaskCode, RcsMode, ChassisCmd


class RcsAction:
    def __init__(self, node):
        self.chassis = node.create_publisher(ChassisCmd, "/rcs_action/chassis", 10)
        self.blocks = node.create_publisher(RcsDetections, "/rcs_action/blocks", 10)
        self.rings = node.create_publisher(RcsDetections, "/rcs_action/rings", 10)
        self.codes = node.create_publisher(RcsTaskCode, "/rcs_action/task_code/send", 10)
        # 模式属于持久状态，晚启动的视觉节点也应得到最近一次选择。
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.modes = node.create_publisher(RcsMode, "/rcs_vision/detection_mode", qos)

    def set_mode(self, mode):
        self.modes.publish(RcsMode(mode=mode))

    def send_code(self, raw):
        self.codes.publish(RcsTaskCode(raw=raw))

    def send_detections(self, msg):
        (self.blocks if msg.mode == 0 else self.rings).publish(msg)

    def send_chassis(self, msg):
        self.chassis.publish(msg)
