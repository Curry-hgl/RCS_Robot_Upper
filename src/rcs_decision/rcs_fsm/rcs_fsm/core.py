"""ROS-independent, periodically driven FSM inspired by FSM_Lite."""
from enum import Enum, auto
from typing import Callable, Hashable


class Event(Enum):
    NONE = auto()
    FINISH = auto()
    FAILED = auto()
    PAUSE = auto()


class Machine:
    """One state callback per tick; FINISH transitions on the following tick.

    Call from a single owner thread. Task decisions may change the next state
    after tick() returns. PAUSE and FAILED are results for the task to handle;
    they do not suspend scheduling automatically.
    """

    def __init__(self):
        self.state = None
        self.event = Event.NONE
        self._callbacks: dict[Hashable, Callable[[], Hashable]] = {}
        self._next: dict[Hashable, Hashable] = {}
        self._pending = None
        self._has_pending = False

    def register(self, state, callback):
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
        self._check(state)
        self._check(target)
        self._next[state] = target

    def start(self, state):
        """Reset scheduling only; task owns cancellation/reset of actions."""
        self._check(state)
        self.state = state
        self.event = Event.NONE
        self._has_pending = False

    def jump(self, state):
        """Schedule an explicit transition for the next tick."""
        self._check(state)
        self._pending = state
        self._has_pending = True

    def tick(self):
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
