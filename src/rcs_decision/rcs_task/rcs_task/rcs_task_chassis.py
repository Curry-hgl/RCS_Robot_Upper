"""底盘任务；非阻塞动作与速度指令互斥，不承担电机闭环。"""
from copy import deepcopy
import math

from .rcs_task_base import RcsTaskBase


class RcsTaskChassis(RcsTaskBase):
    def __init__(self, actions):
        super().__init__()
        self.actions = actions
        self._command = None
        self._last_command = None

    def on_command(self, command):
        if self.busy:
            raise RuntimeError('Chassis action is active')
        if not all(math.isfinite(v) for v in (command.vx, command.vy, command.wz)):
            raise ValueError('Chassis velocity must be finite')
        self._command = deepcopy(command)

    def start(self, action):
        if self._command is not None or self._last_command is not None:
            raise RuntimeError('Stop the chassis velocity command before starting an action')
        super().start(action)

    def tick(self):
        if self._command is not None:
            self.actions.send_chassis(self._command)
            self._last_command = self._command if any(
                (self._command.vx, self._command.vy, self._command.wz)) else None
            self._command = None
        return super().tick()

    def cancel(self):
        self._command = None
        if self._last_command is not None:
            stop = deepcopy(self._last_command)
            stop.vx = stop.vy = stop.wz = 0.0
            self.actions.send_chassis(stop)
            self._last_command = None
        super().cancel()
