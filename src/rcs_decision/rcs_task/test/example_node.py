"""ROS timer adapter; all task state is owned by the executor thread."""
import rclpy
from rclpy.node import Node
from example_task import ExampleTask, State, EXAMPLE_LIFT, lift_parameters


class ExampleTaskNode(Node):
    def __init__(self):
        super().__init__("example_task")
        period = float(self.declare_parameter("tick_period", 0.05).value)
        if period <= 0:
            raise ValueError("tick_period must be positive")
        simulate = self.declare_parameter("simulate", True).value
        action = None
        if not simulate:
            from rcs_action.rcs_action_ros_adapter import RosAction
            height = float(self.declare_parameter("target_height", 0.1).value)
            action = RosAction(self, EXAMPLE_LIFT, lift_parameters(height))
        self.task = ExampleTask(action)
        self.task.request_start()
        self.timer = self.create_timer(period, self._tick)

    def _tick(self):
        if (self.task.machine.state == State.IDLE
                and not getattr(self.task.action, "ready", True)):
            return
        before = self.task.machine.state
        self.task.tick()
        after = self.task.machine.state
        if before != after:
            self.get_logger().info(f"{before.name} -> {after.name}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExampleTaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.task.reset()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
