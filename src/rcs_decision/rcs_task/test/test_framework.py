import unittest
from rcs_fsm import Event, Machine
from rcs_action import SimulatedAction, Status
from example_task import ExampleTask, State


class CountingAction(SimulatedAction):
    starts = 0

    def start(self):
        self.starts += 1
        super().start()


class FrameworkTests(unittest.TestCase):
    def test_decision_can_change_destination_before_next_tick(self):
        fsm = Machine()
        for name in ('a', 'b', 'c'):
            fsm.register(name, lambda: Event.FINISH)
        fsm.set_next('a', 'b')
        fsm.start('a')
        self.assertEqual(fsm.tick(), Event.FINISH)
        self.assertEqual(fsm.state, 'a')
        fsm.set_next('a', 'c')
        fsm.tick()
        self.assertEqual(fsm.state, 'c')

    def test_send_once_and_restart(self):
        action = CountingAction(3)
        task = ExampleTask(action)
        task.request_start()
        for _ in range(5):
            task.tick()
        self.assertEqual(task.machine.state, State.DONE)
        self.assertEqual(action.starts, 1)
        task.reset()
        task.request_start()
        task.tick()
        task.tick()
        self.assertEqual(action.starts, 2)
        task.reset()
        self.assertEqual(action.poll(), Status.CANCELLED)
        self.assertEqual(task.machine.state, State.IDLE)

    def test_failure_is_task_policy(self):
        class FailedAction(SimulatedAction):
            def poll(self):
                return Status.FAILED
        task = ExampleTask(FailedAction())
        task.request_start()
        task.tick()
        self.assertEqual(task.tick(), Event.FAILED)
        self.assertEqual(task.machine.state, State.EXECUTE)
        task.tick()
        self.assertEqual(task.machine.state, State.ERROR)

    def test_invalid_configuration_is_reported(self):
        fsm = Machine()
        with self.assertRaises(RuntimeError):
            fsm.tick()
        fsm.register('a', lambda: Event.FINISH)
        with self.assertRaises(ValueError):
            fsm.set_next('a', 'missing')
        fsm.start('a')
        fsm.tick()
        with self.assertRaises(RuntimeError):
            fsm.tick()


if __name__ == '__main__':
    unittest.main()
