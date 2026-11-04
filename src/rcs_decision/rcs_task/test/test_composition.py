"""任务隔离、组合调度和非阻塞动作回归。"""
import unittest
from types import SimpleNamespace as NS

from rcs_action import SimulatedAction, Status
from rcs_fsm import Event
from rcs_task.rcs_task import RcsTask, select_tasks
from rcs_task.rcs_task_base import TaskState


class Actions:
    def __init__(self):
        self.commands, self.modes = [], []
    def send_chassis(self, msg): self.commands.append(msg)
    def set_mode(self, mode): self.modes.append(mode)
    def send_code(self, code): pass
    def send_detections(self, msg): pass


class CountingAction(SimulatedAction):
    def __init__(self, polls=3):
        super().__init__(polls)
        self.starts = 0
        self.ready = True
    def start(self):
        self.starts += 1
        super().start()


class CompositionTests(unittest.TestCase):
    def test_single_task_does_not_create_other_tasks(self):
        for name in ('upper', 'chassis', 'vision'):
            actions = Actions()
            task = RcsTask(actions, lambda: 0, tasks=[name], qr_enabled=False)
            self.assertEqual(list(task.tasks), [name])
            self.assertEqual(list(task.tick()), [name])
            self.assertEqual(actions.modes, [0] if name == 'vision' else [])
            self.assertEqual(task.status()['pick_count'], 0)
            self.assertTrue(task.status()['phase'].startswith(name + ':'))

    def test_composed_actions_progress_independently_and_send_once(self):
        task = RcsTask(Actions(), lambda: 0, tasks=['upper', 'chassis'])
        upper, chassis = task.tasks['upper'], task.tasks['chassis']
        slow, fast = CountingAction(4), CountingAction(1)
        upper.start(slow)
        chassis.start(fast)
        task.tick()
        task.tick()
        self.assertTrue(upper.busy)
        self.assertEqual(chassis.machine.state, TaskState.DONE)
        self.assertEqual(task.tasks['upper'].result, None)
        for _ in range(5): task.tick()
        self.assertEqual(upper.machine.state, TaskState.DONE)
        self.assertEqual((slow.starts, fast.starts), (1, 1))

    def test_decision_runs_after_task_results_and_can_start_next_action(self):
        observed = []
        second = CountingAction(1)
        def decision(task, events):
            observed.append(events.copy())
            if events['upper'] == Event.FINISH:
                task.tasks['chassis'].start(second)
        task = RcsTask(Actions(), lambda: 0, tasks=['upper', 'chassis'], decision=decision)
        task.tasks['upper'].start(CountingAction(1))
        task.tick()
        self.assertEqual(second.starts, 0)
        task.tick()
        self.assertEqual(second.starts, 1)
        self.assertEqual(observed[0]['upper'], Event.FINISH)

    def test_action_waits_for_connection_and_cancellation_is_confirmed(self):
        task = RcsTask(Actions(), lambda: 0, tasks=['upper'])
        upper = task.tasks['upper']
        action = CountingAction()
        action.ready = False
        upper.start(action)
        task.tick()
        self.assertEqual(action.starts, 0)
        with self.assertRaises(RuntimeError): upper.start(CountingAction())
        upper.cancel()
        task.tick()
        self.assertEqual(upper.machine.state, TaskState.CANCELLED)
        action.ready = True
        upper.start(action)
        task.tick()
        upper.cancel()
        task.tick()
        task.tick()
        self.assertEqual(upper.machine.state, TaskState.CANCELLED)
        self.assertEqual(action.starts, 1)

    def test_action_failure_is_not_success(self):
        task = RcsTask(Actions(), lambda: 0, tasks=['chassis'])
        class Failure(CountingAction):
            def poll(self): return Status.FAILED
        chassis = task.tasks['chassis']
        chassis.start(Failure())
        self.assertEqual(task.tick()['chassis'], Event.FAILED)
        task.tick()
        self.assertEqual(chassis.machine.state, TaskState.FAILED)

    def test_chassis_velocity_and_action_cannot_overlap(self):
        actions = Actions()
        task = RcsTask(actions, lambda: 0, tasks=['chassis'])
        chassis = task.tasks['chassis']
        command = NS(vx=0.1, vy=0.0, wz=0.0)
        chassis.on_command(command)
        with self.assertRaises(RuntimeError): chassis.start(CountingAction())
        task.tick()
        task.tick()
        self.assertEqual(actions.commands, [command])
        with self.assertRaises(RuntimeError): chassis.start(CountingAction())
        chassis.on_command(NS(vx=0.0, vy=0.0, wz=0.0))
        task.tick()
        chassis.start(CountingAction())
        with self.assertRaises(RuntimeError): chassis.on_command(command)
        with self.assertRaises(ValueError):
            RcsTask(actions, lambda: 0, tasks=['chassis']).tasks['chassis'].on_command(
                NS(vx=float('nan'), vy=0.0, wz=0.0))

    def test_task_selection_rejects_unknown_empty_and_duplicate_names(self):
        self.assertEqual(select_tasks('upper,vision'), ('upper', 'vision'))
        for names in ([], ['navigation'], ['upper', 'upper'], ''):
            with self.assertRaises(ValueError): select_tasks(names)

    def test_closing_chassis_clears_pending_command_and_stops_active_velocity(self):
        actions = Actions()
        task = RcsTask(actions, lambda: 0, tasks=['chassis'])
        chassis = task.tasks['chassis']
        chassis.on_command(NS(vx=0.1, vy=0.0, wz=0.0))
        task.close()
        task.tick()
        self.assertEqual(actions.commands, [])
        chassis.on_command(NS(vx=0.1, vy=0.0, wz=0.0))
        task.tick()
        task.close()
        self.assertEqual(len(actions.commands), 2)
        self.assertEqual(actions.commands[-1].vx, 0)

    def test_vision_only_preserves_feedback_statistics_without_upper_task(self):
        task = RcsTask(Actions(), lambda: 0)
        self.assertEqual(list(task.tasks), ['vision'])
        task.on_gripper(NS(action=5))
        task.on_gripper(NS(action=1))
        task.on_gripper(NS(action=3))
        self.assertEqual((task.status()['pick_count'], task.status()['place_count']), (1, 1))
        combined = RcsTask(Actions(), lambda: 0, tasks=['upper', 'vision'])
        combined.on_gripper(NS(action=1))
        self.assertEqual(combined.tasks['upper'].pick_count, 1)
        self.assertEqual(combined.status()['pick_count'], 1)
