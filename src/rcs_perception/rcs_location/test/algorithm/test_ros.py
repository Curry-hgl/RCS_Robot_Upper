"""模拟 IMU 验证实际节点的发布、断流和归零服务。"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rcs_location.msg import RcsLocationState
from std_srvs.srv import Trigger


def main(length_unit="cm", angle_unit="deg"):
    precision = 2 if length_unit == "cm" else 6
    length_scale = {"m": 1, "cm": 100, "mm": 1000}[length_unit]
    angle_scale = 180 / math.pi if angle_unit == "deg" else 1
    scope = f'/test_rcs_location_{os.getpid()}'
    prefix = Path(get_package_prefix('rcs_location')) / 'lib/rcs_location'
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('location_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(Imu, scope + '/imu', 10)
    odometry, readiness, states = [], [], []
    node.create_subscription(Odometry, scope + '/odom', odometry.append, 10)
    node.create_subscription(Bool, scope + '/rcs_location_algorithm/ready',
                             lambda msg: readiness.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(RcsLocationState, scope + '/state', states.append, 10)
    reset = node.create_client(Trigger, scope + '/rcs_location_algorithm/reset')
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            str(prefix / 'rcs_location_algorithm_node'), '--ros-args',
            '-r', f'__ns:={scope}', '-p', f'imu_topic:={scope}/imu',
            '-p', f'odom_topic:={scope}/odom', '-p', f'state_topic:={scope}/state',
            '-p', f'length_unit:={length_unit}', '-p', f'angle_unit:={angle_unit}',
            '-p', f'state_precision:={precision}',
            '-p', 'calibration_seconds:=0.2',
            '-p', 'calibration_samples:=10', '-p', 'input_timeout:=0.3'],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            def spin_until(predicate, seconds=5):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    if process.poll() is not None:
                        raise AssertionError('algorithm exited unexpectedly')
                    executor.spin_once(timeout_sec=0.01)
                    if predicate():
                        return
                raise AssertionError('ROS condition timed out')

            spin_until(lambda: publisher.get_subscription_count() > 0 and reset.service_is_ready())

            def send(seconds, acceleration=0.0, yaw=0.0):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    msg = Imu()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'
                    msg.orientation.w = math.cos(yaw / 2)
                    msg.orientation.z = math.sin(yaw / 2)
                    msg.linear_acceleration.x = -acceleration
                    msg.linear_acceleration.z = -9.80665
                    publisher.publish(msg)
                    tick_end = time.monotonic() + 0.01
                    while time.monotonic() < tick_end:
                        executor.spin_once(timeout_sec=0.001)

            send(0.7)
            spin_until(lambda: bool(odometry) and bool(readiness) and readiness[-1])
            assert abs(odometry[-1].pose.pose.position.x) < 1e-6
            send(0.15, yaw=0.3)
            spin_until(lambda: states and math.isclose(states[-1].yaw, round(0.3 * angle_scale, precision), abs_tol=1e-6))
            assert states[-1].length_unit == length_unit
            assert states[-1].angle_unit == angle_unit
            assert math.isclose(odometry[-1].pose.pose.orientation.z, math.sin(.15), rel_tol=1e-8)
            send(0.6, 1.0)
            spin_until(lambda: odometry[-1].twist.twist.linear.x > 0.3)
            assert odometry[-1].pose.pose.position.x > 0.04
            assert odometry[-1].child_frame_id == 'base_link'
            assert odometry[-1].pose.covariance[0] > 0
            spin_until(lambda: states and states[-1].vx / length_scale > .3)
            state = states[-1]
            spin_until(lambda: any(math.isclose(state.x / length_scale, item.pose.pose.position.x, abs_tol=0.51 * 10**(-precision) / length_scale)
                       and math.isclose(state.vx / length_scale, item.twist.twist.linear.x, abs_tol=0.51 * 10**(-precision) / length_scale)
                       for item in odometry))
            spin_until(lambda: readiness and not readiness[-1])
            before = len(odometry)
            send(0.05)
            spin_until(lambda: len(odometry) > before)
            assert abs(odometry[-1].twist.twist.linear.x) < 1e-6
            future = reset.call_async(Trigger.Request())
            spin_until(future.done)
            assert future.result().success
            send(0.7)
            spin_until(lambda: readiness[-1] and abs(odometry[-1].pose.pose.position.x) < 1e-6)
            assert math.isclose(odometry[-1].pose.pose.orientation.w, 1.0)
            print(f'ROS integration: {length_unit}/{angle_unit}, SI odometry, timeout, gap and reset passed')
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)


if __name__ == '__main__':
    for units in [('cm', 'deg'), ('m', 'rad'), ('mm', 'deg')]:
        main(*units)
