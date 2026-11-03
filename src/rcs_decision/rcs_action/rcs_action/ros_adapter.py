"""远程动作的 ROS 适配：仅映射消息字段，字节编码统一归 rcs_msgs。"""
from rcs_msgs.msg import ActionCommand, ActionFeedback
from .remote import RemoteAction


class RosAction(RemoteAction):
    def __init__(self, node, action_code, data=b"", timeout=5.0):
        self.publisher = node.create_publisher(ActionCommand, "/action/command", 10)
        super().__init__(self._publish, action_code, data, timeout)
        self.subscription = node.create_subscription(
            ActionFeedback, "/action/feedback", self.on_feedback, 10)

    @property
    def ready(self):
        return (self.publisher.get_subscription_count() > 0
                and self.subscription.get_publisher_count() > 0)

    def _publish(self, command):
        msg = ActionCommand()
        msg.request_id = command.request_id
        msg.action_code = command.action_code
        msg.operation = command.operation
        msg.data = list(command.data)
        self.publisher.publish(msg)
