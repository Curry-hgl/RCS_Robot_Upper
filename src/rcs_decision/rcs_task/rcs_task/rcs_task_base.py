"""独立任务的周期入口和非阻塞动作状态机。"""
from enum import Enum, auto

from rcs_action import Status
from rcs_fsm import Event, Machine


class TaskState(Enum):
    IDLE = auto()
    EXECUTE = auto()
    DONE = auto()
    FAILED = auto()
    CANCELLED = auto()


class RcsTaskBase:
    def __init__(self):
        self.action = None
        self.result = None
        self._started = False
        self.machine = Machine()
        for state in TaskState:
            self.machine.register(state, self._execute if state == TaskState.EXECUTE
                                  else lambda: Event.NONE)
        self.machine.set_next(TaskState.EXECUTE, TaskState.DONE)
        self.machine.start(TaskState.IDLE)

    @property
    def busy(self):
        return self.action is not None and self.result is None

    def start(self, action):
        """由任务状态函数或比赛决策提交动作；执行中不接受覆盖。"""
        if self.busy:
            raise RuntimeError('Task already has an active action')
        self.action, self.result, self._started = action, None, False
        self.machine.start(TaskState.EXECUTE)

    def cancel(self):
        if self.busy:
            if self._started:
                self.action.cancel()
            else:
                self.result = Status.CANCELLED
                self.machine.jump(TaskState.CANCELLED)

    def _execute(self):
        if not self._started:
            if not getattr(self.action, 'ready', True):
                return Event.NONE
            try:
                self.action.start()
            except Exception:
                self.result = Status.FAILED
                self.machine.jump(TaskState.FAILED)
                raise
            self._started = True
        result = self.action.poll()
        if result == Status.RUNNING:
            return Event.NONE
        self.result = result
        if result == Status.SUCCEEDED:
            return Event.FINISH
        self.machine.jump(TaskState.CANCELLED if result == Status.CANCELLED else TaskState.FAILED)
        return Event.PAUSE if result == Status.CANCELLED else Event.FAILED

    def tick(self):
        return self.machine.tick()
