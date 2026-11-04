"""Lite 风格纯 Python 内核：只调度注册函数，不依赖 ROS、动作或赛题。"""
from enum import Enum, auto
from typing import Callable, Hashable


class Event(Enum):
    NONE = auto()
    FINISH = auto()
    FAILED = auto()
    PAUSE = auto()


class Machine:
    """每周期执行一个状态函数，FINISH 在下一周期触发跳转。

    由单一线程驱动。tick 返回后，任务可修改后继表；PAUSE/FAILED 交给
    任务处理，本身不会停止调度。所有比赛条件判断均留在 rcs_task。
    """

    def __init__(self):
        self.state = None
        self.event = Event.NONE
        # 状态函数表与后继表独立，执行机制无需理解业务决策。
        self._callbacks: dict[Hashable, Callable[[], Hashable]] = {}
        self._next: dict[Hashable, Hashable] = {}
        self._pending = None
        self._has_pending = False

    def register(self, state, callback):
        """注册唯一状态及无参执行函数；函数每周期返回一个事件。"""
        if state is None:
            raise ValueError("None is reserved for an unstarted machine")
        if state in self._callbacks:
            raise ValueError(f"State already registered: {state}")
        if not callable(callback):
            raise TypeError("State callback must be callable")
        self._callbacks[state] = callback

    def _check(self, state):
        if state not in self._callbacks:
            raise ValueError(f"Unregistered state: {state}")

    def set_next(self, state, target):
        """设置 FINISH 后继；任务可在本周期执行后、下周期跳转前修改。"""
        self._check(state)
        self._check(target)
        self._next[state] = target

    def start(self, state):
        """重置调度至指定状态；动作取消和业务清理由任务负责。"""
        self._check(state)
        self.state = state
        self.event = Event.NONE
        self._has_pending = False

    def jump(self, state):
        """预约下一周期直接跳转，优先于普通完成跳转。"""
        self._check(state)
        self._pending = state
        self._has_pending = True

    def tick(self):
        """先处理上一周期跳转，再调用一次当前状态函数，返回本周期事件。"""
        if self.state is None:
            raise RuntimeError("Machine has not been started")
        if self._has_pending:
            self.state = self._pending
            self._has_pending = False
            self.event = Event.NONE
        elif self.event == Event.FINISH:
            if self.state not in self._next:
                raise RuntimeError(f"No next state configured for {self.state}")
            self.state = self._next[self.state]
            self.event = Event.NONE
        self.event = self._callbacks[self.state]()
        return self.event
