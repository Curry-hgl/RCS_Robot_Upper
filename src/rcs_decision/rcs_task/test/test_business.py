import unittest
from rcs_msgs.business import (Command, Feedback, encode_command, decode_command,
                               encode_feedback, decode_feedback)
from example_task import lift_parameters
from rcs_action.rcs_action_remote import RemoteAction
from rcs_action import Status
from rcs_fsm import Event
from example_task import ExampleTask, State
from rcs_ptl import FrameCodec


class BusinessTests(unittest.TestCase):
    def test_wire_vectors(self):
        command = Command(1, 1, data=lift_parameters(0.1))
        self.assertEqual(encode_command(command).hex(), '100101000000010000cdcccc3d')
        self.assertEqual(decode_command(encode_command(command)), command)
        feedback = Feedback(1, 1, 2)
        self.assertEqual(encode_feedback(feedback).hex(), '1101010000000100020000')
        self.assertEqual(decode_feedback(encode_feedback(feedback)), feedback)
        with self.assertRaises(ValueError):
            decode_feedback(b'\x11\x02')
        with self.assertRaises(ValueError):
            encode_command(Command(1, 1, data=b'x' * 117))

    def test_task_to_rdlc_and_back(self):
        outbound = []
        codec = FrameCodec(escape_enabled=True)
        action = RemoteAction(lambda c: outbound.append(codec.encode(
            1, 2, encode_command(c))), 1, bytes((0xc0, 0x0c, 0xff)))
        task = ExampleTask(action)
        task.request_start()
        task.tick()
        task.tick()
        task.tick()
        self.assertEqual(len(outbound), 1)
        request = decode_command(codec.feed(outbound[0])[0].payload)
        action.on_feedback(Feedback(request.request_id ^ 1, 1, 2))
        self.assertEqual(action.poll(), Status.RUNNING)
        raw = codec.encode(2, 1, encode_feedback(Feedback(request.request_id, 1, 2)))
        action.on_feedback(decode_feedback(codec.feed(raw)[0].payload))
        self.assertEqual(task.tick(), Event.FINISH)
        task.tick()
        self.assertEqual(task.machine.state, State.DONE)

    def test_timeout_and_cancellation(self):
        commands = []
        now = [0]
        action = RemoteAction(commands.append, 1, timeout=1, clock=lambda: now[0])
        action.start()
        action.cancel()
        self.assertEqual(action.poll(), Status.RUNNING)
        with self.assertRaises(RuntimeError):
            action.start()
        action.on_feedback(Feedback(action.request_id, 1, 4))
        self.assertEqual(action.poll(), Status.CANCELLED)
        action.start()
        now[0] = 2
        self.assertEqual(action.poll(), Status.FAILED)
        self.assertEqual(commands[-1].operation, 1)
