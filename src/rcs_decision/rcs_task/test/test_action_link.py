import os
import pty
import select
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcs_serial.rcs_serial_bridge import SerialBridge
from rcs_serial.rcs_serial_config import SerialConfig
from example_node import ExampleTaskNode
from example_task import State
from rcs_ptl import FrameCodec
from rcs_msgs.business import decode_command, encode_feedback, Feedback

def test_action_link():
    master, slave = pty.openpty()
    stop = threading.Event()
    seen = []
    errors = []
    def mcu():
        codec = FrameCodec(escape_enabled=True)
        try:
            while not stop.is_set():
                if not select.select([master], [], [], 0.05)[0]:
                    continue
                for frame in codec.feed(os.read(master, 1024)):
                    command = decode_command(frame.payload)
                    seen.append(command)
                    os.write(master, codec.encode(2, 1, encode_feedback(
                        Feedback(command.request_id, command.action_code, 2))))
        except Exception as exc:
            errors.append(exc)

    rclpy.init(args=['--ros-args', '-p', 'simulate:=false'])
    bridge = SerialBridge(SerialConfig(os.ttyname(slave), escape_enabled=True))
    bridge.open()
    thread = threading.Thread(target=mcu)
    thread.start()
    task = ExampleTaskNode()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(task)
    try:
        deadline = time.monotonic() + 10
        while task.task.machine.state != State.DONE and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert not errors, errors
        assert task.task.machine.state == State.DONE, task.task.machine.state
        assert len(seen) == 1, seen
        print('PASS: task -> ROS -> business codec -> RDLC -> PTY MCU -> feedback -> DONE')
    finally:
        stop.set()
        thread.join()
        bridge.close()
        task.destroy_node()
        bridge.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
