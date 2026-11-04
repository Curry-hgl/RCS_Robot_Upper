"""上层机构任务；机构动作通过 rcs_action 注入。"""
from .rcs_task_base import RcsTaskBase
from .rcs_task_status import GripperStatus


class RcsTaskUpper(RcsTaskBase):
    def __init__(self, *, gripper=None):
        super().__init__()
        self.gripper = GripperStatus() if gripper is None else gripper

    @property
    def pick_count(self):
        return self.gripper.pick_count

    @property
    def place_count(self):
        return self.gripper.place_count

    def on_gripper(self, event):
        self.gripper.on_gripper(event)
