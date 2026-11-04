"""可选状态显示：订阅任务码和统计，窗口关闭不影响任务/通信节点。"""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcs_msgs.msg import RcsStatus, RcsMode, RcsGripperEvent
from .rcs_debug_display import TaskDisplay


class RcsDebugMonitor(Node):
    def __init__(self):
        super().__init__("rcs_debug_monitor")
        fullscreen = self.declare_parameter("fullscreen", False).value
        self.mode = 0
        self.mode_pub = self.create_publisher(RcsMode, "/rcs_task/mode_command", 10)
        self.count_pub = self.create_publisher(RcsGripperEvent, "/rcs_task/gripper_event", 10)
        self.display = TaskDisplay(width=1280, height=720, fullscreen=fullscreen)
        cv2.namedWindow(self.display.window_name, cv2.WINDOW_NORMAL)
        if fullscreen:
            cv2.setWindowProperty(self.display.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(RcsStatus, "/rcs_task/status", self._update, qos)
        self.timer = self.create_timer(0.1, self._draw)

    def _update(self, msg):
        self.mode = msg.mode
        self.display.set_task_code(msg.task_code)
        self.display.set_counts(msg.pick_count, msg.place_count)

    def _draw(self):
        cv2.imshow(self.display.window_name, self.display.render_frame())
        key = cv2.waitKey(1) & 0xff
        # 调试按键同样走 ROS 事件，不绕过任务直接修改显示计数。
        operations = {ord("1"): 1, ord("+"): 1, ord("="): 1,
                      ord("3"): 2, ord("-"): 2, ord("_"): 2,
                      ord("2"): 3, ord("]"): 3,
                      ord("4"): 4, ord("["): 4}
        if key in operations:
            self.count_pub.publish(RcsGripperEvent(action=operations[key]))
        elif key == ord("m"):
            self.mode_pub.publish(RcsMode(mode=1 - self.mode))
        elif key == ord("f"):
            self.display.fullscreen = not self.display.fullscreen
            cv2.setWindowProperty(self.display.window_name, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if self.display.fullscreen else cv2.WINDOW_NORMAL)
        elif key == 27:
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RcsDebugMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
