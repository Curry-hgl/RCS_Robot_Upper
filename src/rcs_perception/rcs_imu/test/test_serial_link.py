"""伪串口执行真实 SDK 握手、ROS 消息发布、服务和退出。"""
import math
import os
from pathlib import Path
import random
import select
import signal
import struct
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
import yaml


FLAGS = (1 << 1) | (1 << 7) | (1 << 11)


def packet(command, payload=b''):
    header = struct.pack('<HHH', 1, command, len(payload))
    return b':' + header + payload + struct.pack('<H', (sum(header) + sum(payload)) & 65535) + b'\r\n'


class Emulator:
    def __init__(self):
        self.master, self.slave = os.openpty()
        self.port = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.commands = []
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = bytearray()
        streaming = False
        stamp = 0
        while not self.stop.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.01)
            if ready:
                buffer.extend(os.read(self.master, 4096))
            while len(buffer) >= 7:
                if buffer[0] != 0x3a:
                    del buffer[0]
                    continue
                _, command, size = struct.unpack_from('<HHH', buffer, 1)
                if len(buffer) < size + 11:
                    break
                frame = bytes(buffer[:size+11])
                del buffer[:size+11]
                assert frame[-2:] == b'\r\n'
                assert sum(frame[1:-4]) & 65535 == int.from_bytes(frame[-4:-2], 'little')
                self.commands.append(command)
                if command in (6, 7, 38, 62):
                    if command in (6, 7): streaming = command == 7
                    reply = packet(0)
                else:
                    text = {20: b'LPMS-BE-S102', 21: b'BE-S102-1.0',
                            22: b'SIMULATED-IMU', 23: b'1.0'}
                    values = {24: 1, 31: FLAGS, 33: 1, 35: 100, 37: 0,
                              51: 2, 61: 2000, 65: 1, 131: 115200, 137: 1}
                    payload = text.get(command, struct.pack('<I', values.get(command, 0)))
                    if command == 161: payload = b'\x00'*8
                    reply = packet(command, payload)
                os.write(self.master, reply)
            if streaming:
                stamp += 1
                # timestamp, acc(g), gyro(deg/s), quaternion(wxyz)
                data = struct.pack('<I10f', stamp, 0, 0, -1, 90, 0, 0, 1, 0, 0, 0)
                good = packet(9, data)
                if stamp == 1:
                    corrupt = bytearray(good); corrupt[-4] ^= 1
                    os.write(self.master, corrupt)
                os.write(self.master, good)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


def test_serial_handshake_publish_service_and_shutdown(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(random.randrange(100, 180)))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(context=context)
    observer = rclpy.create_node('imu_test_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    messages = []
    observer.create_subscription(Imu, '/rcs_imu/data', messages.append, qos_profile_sensor_data)
    client = observer.create_client(Trigger, '/rcs_imu/reset_heading')
    emulator = Emulator()
    process = None
    try:
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'imu.yaml'
            config.write_text(yaml.safe_dump({'rcs_imu': {'ros__parameters': {
                'port': emulator.port, 'baudrate': 115200, 'rate': 200.0,
                'debug_enabled': True, 'connection_timeout': 5.0}}}))
            with (Path(folder) / 'output.log').open('w+') as log:
                env = dict(os.environ, ROS_LOG_DIR=folder)
                process = subprocess.Popen(['ros2', 'run', 'rcs_imu', 'rcs_imu', '--config', str(config)],
                                           env=env, stdout=log, stderr=log, start_new_session=True)
                deadline = time.monotonic() + 20
                while len(messages) < 5 and time.monotonic() < deadline and process.poll() is None:
                    executor.spin_once(timeout_sec=0.1)
                log.flush(); log.seek(0)
                assert len(messages) >= 5, log.read()
                message = messages[-1]
                assert message.header.frame_id == 'imu_link'
                assert math.isclose(message.orientation.w, 1, abs_tol=1e-5)
                assert math.isclose(message.linear_acceleration.z, 9.80665, abs_tol=1e-4)
                assert math.isclose(message.angular_velocity.x, math.pi/2, abs_tol=1e-5)
                assert client.wait_for_service(timeout_sec=3)
                future = client.call_async(Trigger.Request())
                deadline = time.monotonic() + 5
                while not future.done() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.1)
                assert future.done() and future.result().success
                while 38 not in emulator.commands and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.1)
                assert 38 in emulator.commands
                os.killpg(process.pid, signal.SIGINT)
                assert process.wait(timeout=8) == 0
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
        emulator.close()
        executor.shutdown()
        observer.destroy_node()
        context.shutdown()
