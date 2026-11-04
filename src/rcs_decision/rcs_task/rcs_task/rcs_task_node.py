"""ROS 接入：仅订阅所选任务的输入，由定时器逐周期驱动。"""
import argparse
import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from rcs_action.rcs_action import RcsAction
from rcs_msgs.msg import ChassisCmd, RcsTaskCode, RcsMode, RcsDetections, RcsGripperEvent, RcsStatus
from .rcs_task import RcsTask, select_tasks
from .rcs_task_config import load_task_config


class RcsTaskNode(Node):
    def __init__(self, *, tasks=None, config=None):
        super().__init__('rcs_task')
        if config is None:
            selected = select_tasks(tasks or ['vision'])
            config = load_task_config('vision' if 'vision' in selected else selected[0])

        def param(name, default):
            return self.declare_parameter(name, config.get(name, default)).value

        configured = self.declare_parameter('tasks', ','.join(config.get('tasks', ['vision']))).value
        selected = select_tasks(configured if tasks is None else tasks)
        vision_options = {}
        if 'vision' in selected:
            vision_options = dict(
                qr_enabled=param('qr_enabled', True),
                send_interval=param('send_interval', 0.5),
                broadcast_duration=param('qr_send_duration', 5.0),
                initial_mode=param('initial_mode', 0),
                qr_confirm_frames=param('qr_confirm_frames', 3),
            )
        self.task = RcsTask(RcsAction(self), time.monotonic, tasks=selected, **vision_options)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(RcsStatus, '/rcs_task/status', qos)
        self.subs = []
        if {'upper', 'vision'} & self.task.tasks.keys():
            self.subs.append(self.create_subscription(
                RcsGripperEvent, '/rcs_task/gripper_event', self.task.on_gripper, 10))
        if 'chassis' in self.task.tasks:
            self.subs.append(self.create_subscription(
                ChassisCmd, '/rcs_task/chassis/command', self._chassis_command, 10))
        if 'vision' in self.task.tasks:
            vision = self.task.tasks['vision']
            self.subs.extend([
                self.create_subscription(RcsTaskCode, '/rcs_vision/qr_candidate',
                                         lambda m: vision.on_qr(m.raw), 10),
                self.create_subscription(RcsMode, '/rcs_task/mode_command',
                                         lambda m: vision.on_mode(m.mode), 10),
                self.create_subscription(RcsDetections, '/rcs_vision/detections',
                                         vision.on_detections, 10),
            ])
        period = param('tick_period', 0.05)
        if not math.isfinite(period) or period <= 0:
            raise ValueError('tick_period must be positive')
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info('Tasks: ' + ', '.join(selected))

    def _chassis_command(self, msg):
        try:
            self.task.tasks['chassis'].on_command(msg)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warning(str(exc))

    def _tick(self):
        self.task.tick()
        self.status.publish(RcsStatus(**self.task.status()))


def parse_args(argv):
    parser = argparse.ArgumentParser(description='任务节点（由任务 launch 调用）')
    parser.add_argument('tasks', nargs='*', metavar='TASK',
                        help='upper chassis vision；省略时读取配置')
    parser.add_argument('--config', help='任务配置路径')
    options = parser.parse_args(remove_ros_args(argv)[1:])
    if options.tasks:
        try:
            options.tasks = select_tasks(options.tasks)
        except ValueError as exc:
            parser.error(str(exc))
    return options


def main(args=None):
    argv = sys.argv if args is None else ['rcs_task', *args]
    options = parse_args(argv)
    selected = options.tasks or ('vision',)
    if len(selected) != 1:
        raise ValueError('Use the rcs_task launcher to combine independent task nodes')
    config = load_task_config(selected[0], options.config)
    rclpy.init(args=argv)
    node = None
    try:
        node = RcsTaskNode(tasks=options.tasks or None, config=config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.task.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
