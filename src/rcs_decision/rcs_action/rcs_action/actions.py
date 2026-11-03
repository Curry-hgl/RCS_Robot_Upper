"""Nonblocking action lifecycle, independent of task flow and FSM events."""
from enum import Enum, auto
from typing import Protocol


class Status(Enum):
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()


class Action(Protocol):
    """Implement send-once and feedback tracking behind this interface.

    start() begins a fresh invocation. poll() never blocks or resends start.
    cancel() requests stopping execution; hardware implementations must track
    cancellation acknowledgement themselves.
    """
    def start(self) -> None: ...
    def poll(self) -> Status: ...
    def cancel(self) -> None: ...


class SimulatedAction:
    """Replaceable example: succeeds after a configured number of polls."""

    def __init__(self, polls=3):
        if polls < 1:
            raise ValueError("polls must be positive")
        self._polls = polls
        self._remaining = 0
        self._status = None

    def start(self):
        self._remaining = self._polls
        self._status = Status.RUNNING

    def poll(self):
        if self._status is None:
            raise RuntimeError("Action has not been started")
        if self._status == Status.RUNNING:
            self._remaining -= 1
            if self._remaining == 0:
                self._status = Status.SUCCEEDED
        return self._status

    def cancel(self):
        if self._status == Status.RUNNING:
            self._status = Status.CANCELLED
