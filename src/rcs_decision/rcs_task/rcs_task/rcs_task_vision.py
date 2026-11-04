"""视觉任务：识别请求、模式选择、二维码确认与结果发送。"""
from enum import Enum, auto
from rcs_fsm import Machine, Event
from rcs_msgs.business.rcs_msgs_payload import valid_task_code


class Phase(Enum):
    SCAN = auto()
    BROADCAST = auto()
    TRACK = auto()


class RcsTaskVision:
    def __init__(self, actions, clock, qr_enabled=True, send_interval=0.5,
                 broadcast_duration=5.0, initial_mode=0, qr_confirm_frames=3):
        if send_interval <= 0 or broadcast_duration < 0 or initial_mode not in (0, 1):
            raise ValueError("Invalid task timing or initial mode")
        if type(qr_confirm_frames) is not int or qr_confirm_frames < 1:
            raise ValueError("qr_confirm_frames must be a positive integer")
        self.confirm_frames = qr_confirm_frames
        self.actions, self.clock = actions, clock
        self.interval, self.duration = send_interval, broadcast_duration
        self.mode = initial_mode
        self.code = ""
        self.candidate, self.matches = "", 0
        self.latest = None
        self.last_send = float("-inf")
        self.broadcast_until = 0
        self.machine = Machine()
        self.machine.register(Phase.SCAN, self._scan)
        self.machine.register(Phase.BROADCAST, self._broadcast)
        self.machine.register(Phase.TRACK, lambda: Event.NONE)
        self.machine.set_next(Phase.SCAN, Phase.BROADCAST)
        self.machine.set_next(Phase.BROADCAST, Phase.TRACK)
        self.machine.start(Phase.SCAN if qr_enabled else Phase.TRACK)
        self.actions.set_mode(self.mode)

    def on_qr(self, raw):
        """每个新相机帧最多调用一次；空值/不同值打断连续确认。"""
        if self.machine.state != Phase.SCAN or self.code:
            return
        if not valid_task_code(raw):
            self.candidate, self.matches = "", 0
            return
        self.matches = self.matches + 1 if raw == self.candidate else 1
        self.candidate = raw
        if self.matches >= self.confirm_frames:
            self.code = raw
            self.actions.send_code(raw)
            self.broadcast_until = self.clock() + self.duration

    def on_mode(self, mode):
        if mode in (0, 1):
            self.mode = mode
            self.latest = None  # 不把切换前的旧结果当成当前模式目标。
            self.actions.set_mode(mode)

    def on_detections(self, msg):
        if msg.mode == self.mode:
            self.latest = (self.clock(), msg)

    def _scan(self):
        return Event.FINISH if self.code else Event.NONE

    def _broadcast(self):
        return Event.FINISH if self.clock() >= self.broadcast_until else Event.NONE

    def tick(self):
        event = self.machine.tick()
        now = self.clock()
        if now - self.last_send < self.interval:
            return event
        if self.code and now < self.broadcast_until:
            self.actions.send_code(self.code)
        # 新帧只发送一次；超时帧直接丢弃，避免相机失联后一直发送旧目标。
        if self.latest is not None:
            captured, msg = self.latest
            if now - captured <= max(1.0, 2 * self.interval):
                self.actions.send_detections(msg)
            self.latest = None
        self.last_send = now
        return event
