"""比赛任务注册与组合；每周期先执行任务，再处理任务间决策。"""
from .rcs_task_upper import RcsTaskUpper
from .rcs_task_status import GripperStatus
from .rcs_task_chassis import RcsTaskChassis
from .rcs_task_vision import RcsTaskVision

TASK_NAMES = ('upper', 'chassis', 'vision')


def select_tasks(names):
    if isinstance(names, str):
        names = names.replace(',', ' ').split()
    names = tuple(names)
    if not names or len(set(names)) != len(names) or any(n not in TASK_NAMES for n in names):
        raise ValueError('tasks must select upper, chassis, vision without duplicates')
    return names


class RcsTask:
    def __init__(self, actions, clock, tasks=('vision',), decision=None, **vision_options):
        self.names = select_tasks(tasks)
        self.gripper = GripperStatus()
        factories = {
            'upper': lambda: RcsTaskUpper(gripper=self.gripper),
            'chassis': lambda: RcsTaskChassis(actions),
            'vision': lambda: RcsTaskVision(actions, clock, **vision_options),
        }
        self.tasks = {name: factories[name]() for name in self.names}
        self.decision = decision

    def tick(self):
        events = {name: task.tick() for name, task in self.tasks.items()}
        if self.decision is not None:
            self.decision(self, events)
        return events

    def close(self):
        for task in self.tasks.values():
            cancel = getattr(task, 'cancel', None)
            if cancel is not None:
                cancel()

    def on_gripper(self, event):
        self.gripper.on_gripper(event)

    def status(self):
        vision = self.tasks.get('vision')
        return dict(
            task_code=vision.code if vision else '',
            pick_count=self.gripper.pick_count,
            place_count=self.gripper.place_count,
            mode=vision.mode if vision else 0,
            phase=' | '.join(f'{name}:{task.machine.state.name}'
                             for name, task in self.tasks.items()),
        )
