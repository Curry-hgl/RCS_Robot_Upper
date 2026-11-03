"""Feedback-tracked action; transport injected, no FSM or ROS dependency."""
import secrets
from time import monotonic
from rcs_msgs.business import Command
from .actions import Status


class RemoteAction:
    def __init__(self, send, action_code, data=b"", timeout=5.0, clock=monotonic):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.send = send
        self.action_code = action_code
        self.data = bytes(data)
        self.timeout = timeout
        self.clock = clock
        self.request_id = None
        self._status = None
        self._deadline = 0
        self.feedback = None

    def start(self):
        if self._status == Status.RUNNING:
            raise RuntimeError("Previous action still active")
        self.request_id = secrets.randbelow(0xffffffff) + 1
        self.feedback = None
        self._status = Status.RUNNING
        self._deadline = self.clock() + self.timeout
        try:
            self.send(Command(self.request_id, self.action_code, 0, self.data))
        except Exception:
            self._status = Status.FAILED
            raise

    def on_feedback(self, msg):
        if (self._status != Status.RUNNING or msg.request_id != self.request_id
                or msg.action_code != self.action_code):
            return
        self.feedback = msg
        self._status = {2: Status.SUCCEEDED, 3: Status.FAILED,
                        4: Status.CANCELLED}.get(msg.status, Status.RUNNING)

    def poll(self):
        if self._status is None:
            raise RuntimeError("Action has not been started")
        if self._status == Status.RUNNING and self.clock() >= self._deadline:
            self.send(Command(self.request_id, self.action_code, 1))
            self._status = Status.FAILED
        return self._status

    def cancel(self):
        if self._status == Status.RUNNING:
            self.send(Command(self.request_id, self.action_code, 1))
            # Remain active until cancellation/terminal feedback or timeout.
