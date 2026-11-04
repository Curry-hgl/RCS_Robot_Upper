"""远程动作生命周期：注入发送函数，通过请求编号关联反馈，不依赖 FSM。"""
import secrets
from time import monotonic
from rcs_msgs.business import Command
from .rcs_action_base import Status


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
        """生成新请求编号并发送一次；执行中禁止重复启动。"""
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
        """仅接受本次动作反馈，忽略其他动作或延迟到达的旧反馈。"""
        if (self._status != Status.RUNNING or msg.request_id != self.request_id
                or msg.action_code != self.action_code):
            return
        self.feedback = msg
        self._status = {2: Status.SUCCEEDED, 3: Status.FAILED,
                        4: Status.CANCELLED}.get(msg.status, Status.RUNNING)

    def poll(self):
        """非阻塞检查终态；超时发取消并报告失败，不等于硬件已停止。"""
        if self._status is None:
            raise RuntimeError("Action has not been started")
        if self._status == Status.RUNNING and self.clock() >= self._deadline:
            self.send(Command(self.request_id, self.action_code, 1))
            self._status = Status.FAILED
        return self._status

    def cancel(self):
        """请求取消；收到下位机终态前仍保持执行中。"""
        if self._status == Status.RUNNING:
            self.send(Command(self.request_id, self.action_code, 1))
            # Remain active until cancellation/terminal feedback or timeout.
