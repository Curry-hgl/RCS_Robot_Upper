"""可替换的最小任务示例：等待启动→动作执行→完成，失败进入 ERROR。"""
import math
import struct
from enum import Enum, auto
from rcs_action import Action, SimulatedAction, Status
from rcs_fsm import Event, Machine


# Replaceable season example: action 1 = lift, argument = target height in metres.
EXAMPLE_LIFT = 1

def lift_parameters(height):
    if not math.isfinite(height) or height < 0:
        raise ValueError("Height must be finite and nonnegative")
    return struct.pack("<f", height)


class State(Enum):
    IDLE = auto()
    EXECUTE = auto()
    DONE = auto()
    ERROR = auto()


class ExampleTask:
    def __init__(self, action: Action | None = None):
        self.action = action if action is not None else SimulatedAction()
        self.machine = Machine()
        self._requested = False
        self._started = False
        self.machine.register(State.IDLE, self._idle)
        self.machine.register(State.EXECUTE, self._execute)
        self.machine.register(State.DONE, lambda: Event.NONE)
        self.machine.register(State.ERROR, lambda: Event.NONE)
        self.machine.set_next(State.IDLE, State.EXECUTE)
        self.machine.set_next(State.EXECUTE, State.DONE)
        self.machine.start(State.IDLE)

    def request_start(self):
        if self.machine.state != State.IDLE:
            raise RuntimeError("Reset the task before starting again")
        self._requested = True

    def _idle(self):
        return Event.FINISH if self._requested else Event.NONE

    def _execute(self):
        """首次进入只启动一次，后续周期轮询动作结果。"""
        if not self._started:
            self.action.start()
            self._started = True
        status = self.action.poll()
        if status == Status.SUCCEEDED:
            return Event.FINISH
        if status in (Status.FAILED, Status.CANCELLED):
            return Event.FAILED
        return Event.NONE

    def tick(self):
        """先执行内核，再按任务策略处理失败，下一周期跳转。"""
        event = self.machine.tick()
        # Task policy, deliberately outside the reusable FSM kernel.
        if event == Event.FAILED:
            self.machine.jump(State.ERROR)
        return event

    def reset(self):
        """请求取消已启动动作并回到待机；真实硬件重新启动前需确认已停止。"""
        if self._started:
            self.action.cancel()
        self._requested = False
        self._started = False
        self.machine.start(State.IDLE)


def main():
    task = ExampleTask()
    task.request_start()
    for _ in range(10):
        event = task.tick()
        print(f"state={task.machine.state.name} event={event.name}")
        if task.machine.state in (State.DONE, State.ERROR):
            break


if __name__ == "__main__":
    main()
