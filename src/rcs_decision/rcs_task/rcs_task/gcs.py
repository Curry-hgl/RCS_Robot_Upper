"""工创任务决策：二维码连续确认、限时广播、下位机模式选择和计数。

核心只依赖注入的功能接口；ROS 节点适配见 gcs_node.py。
"""
from enum import Enum, auto
from rcs_fsm import Machine, Event
from rcs_msgs.business.gcs import valid_task_code


class Phase(Enum):
    SCAN = auto()
    BROADCAST = auto()
    TRACK = auto()


class GcsTask:
    def __init__(self, actions, clock, qr_enabled=True, send_interval=0.5,
                 broadcast_duration=5.0, initial_mode=0):
        if send_interval <= 0 or broadcast_duration < 0 or initial_mode not in (0, 1):
            raise ValueError("Invalid task timing or initial mode")
        self.actions, self.clock = actions, clock
        self.interval, self.duration = send_interval, broadcast_duration
        self.mode = initial_mode
        self.code = ""
        self.candidate, self.matches = "", 0
        self.pick_count = self.place_count = 0
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
        if self.matches >= 3:
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

    def on_gripper(self, event):
        """按到达顺序应用事件，正确处理一次接收中的清零与增减。"""
        if event.action == 0x10:
            self.pick_count, self.place_count = event.pick, event.place
        elif event.action == 5:
            self.pick_count = self.place_count = 0
        elif event.action in (1, 2):
            self.pick_count = max(0, self.pick_count + (1 if event.action == 1 else -1))
        elif event.action in (3, 4):
            self.place_count = max(0, self.place_count + (1 if event.action == 3 else -1))

    def _scan(self):
        return Event.FINISH if self.code else Event.NONE

    def _broadcast(self):
        return Event.FINISH if self.clock() >= self.broadcast_until else Event.NONE

    def tick(self):
        self.machine.tick()
        now = self.clock()
        if now - self.last_send < self.interval:
            return
        if self.code and now < self.broadcast_until:
            self.actions.send_code(self.code)
        # 新帧只发送一次；超时帧直接丢弃，避免相机失联后一直发送旧目标。
        if self.latest is not None:
            captured, msg = self.latest
            if now - captured <= max(1.0, 2 * self.interval):
                self.actions.send_detections(msg)
            self.latest = None
        self.last_send = now
