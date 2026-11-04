"""反馈计数；不依赖机构任务是否启用。"""


class GripperStatus:
    def __init__(self):
        self.pick_count = self.place_count = 0

    def on_gripper(self, event):
        if event.action == 0x10:
            self.pick_count, self.place_count = event.pick, event.place
        elif event.action == 5:
            self.pick_count = self.place_count = 0
        elif event.action in (1, 2):
            self.pick_count = max(0, self.pick_count + (1 if event.action == 1 else -1))
        elif event.action in (3, 4):
            self.place_count = max(0, self.place_count + (1 if event.action == 3 else -1))
