"""RC 国赛视觉节点：只采集、识别和发布，不持有串口或比赛状态机。"""
import argparse
import sys

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from .rcs_vision_config import load_vision_config
from rcs_msgs.msg import RcsDetection, RcsDetections, RcsTaskCode, RcsMode, RcsStatus
from .rcs_vision_camera import Camera
from .rcs_vision_detection import detect_blocks, detect_rings
from .rcs_vision_display import draw_blocks, draw_rings
from .rcs_vision_qr import QrDetector, parse_task_code


class RcsVision(Node):
    def __init__(self, *, config=None):
        config = load_vision_config() if config is None else config
        super().__init__("rcs_vision")
        def param(name, default):
            return self.declare_parameter(name, config.get(name, default)).value
        source = str(param("block_camera", "/dev/video20"))
        qr_source = str(param("qr_camera", "0"))
        width, height = param("width", 640), param("height", 480)
        self.preview = param("preview", False)
        self.qr_preview = param("qr_preview", False)
        self._qr_window_open = False
        self.mode = param("initial_mode", 0)
        if self.mode not in (0, 1):
            self.destroy_node()
            raise ValueError("initial_mode must be 0 or 1")
        self.block_args = dict(min_radius=param("block_min_radius", 20),
            max_radius=param("block_max_radius", 150),
            min_fill_ratio=param("block_min_fill", 0.60),
            min_circularity=param("block_min_circularity", 0.60),
            min_aspect_ratio=param("block_min_aspect", 0.65))
        self.ring_args = dict(min_radius=param("ring_min_radius", 20),
            max_radius=param("ring_max_radius", 200),
            min_color_area=param("ring_min_area", 500), param2=param("ring_param2", 0.9))
        period = float(param("process_period", 0.05))
        if period <= 0:
            raise ValueError("process_period must be positive")
        self.pub = self.create_publisher(RcsDetections, "/rcs_vision/detections", 10)
        self.qr_pub = self.create_publisher(RcsTaskCode, "/rcs_vision/qr_candidate", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_sub = self.create_subscription(RcsMode, "/rcs_vision/detection_mode", self._mode, qos)
        self.status_sub = self.create_subscription(RcsStatus, "/rcs_task/status", self._status, qos)
        self.camera = None
        self.qr_camera = None
        self.qr_detector = None
        try:
            if param("qr_enabled", True):
                self.qr_detector = QrDetector(param("qr_backend", "opencv"),
                                               param("qr_model_dir", "~/.opencv/wechat_qrcode"))
            self.camera = Camera(source, width, height)
            # 同一相机时共享本次帧，不能重复打开同一个设备。
            if self.qr_detector and source != qr_source:
                self.qr_camera = Camera(qr_source, width, height)
            self.timer = self.create_timer(period, self._tick)
        except Exception:
            self.close()
            self.destroy_node()
            raise

    def _status(self, msg):
        # 确认后释放独立二维码相机，主相机共享模式下只停止二维码计算。
        if msg.task_code and self.qr_detector is not None:
            self.qr_detector = None
            if self.qr_camera is not None:
                self.qr_camera.close()
                self.qr_camera = None
            if self._qr_window_open:
                cv2.destroyWindow("RCS QR")
                self._qr_window_open = False

    def _mode(self, msg):
        if msg.mode in (0, 1):
            self.mode = msg.mode

    def _tick(self):
        frame = self.camera.read()
        if self.qr_detector:
            qr_frame = self.qr_camera.read() if self.qr_camera else frame
            if qr_frame is not None:
                texts = self.qr_detector.decode(qr_frame)
                valid = [parse_task_code(text) for text in texts]
                candidate = next((tc.raw for tc in valid if tc.valid), "")
                if self.qr_preview:
                    cv2.imshow("RCS QR", qr_frame)
                    self._qr_window_open = True
                # 空候选也发布，打断连续确认。
                self.qr_pub.publish(RcsTaskCode(raw=candidate))
        if frame is None:
            if self._qr_window_open:
                cv2.waitKey(1)
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        objects = (detect_blocks(hsv, **self.block_args) if self.mode == 0
                   else detect_rings(frame, hsv, **self.ring_args))
        msg = RcsDetections(mode=self.mode)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rcs_vision_camera_pixels"
        msg.objects = [RcsDetection(color_id=o.color_id, cx=float(o.cx),
                                   cy=float(o.cy), radius=float(o.radius)) for o in objects]
        self.pub.publish(msg)
        if self.preview:
            (draw_blocks if self.mode == 0 else draw_rings)(frame, objects)
            cv2.imshow("RCS detection", frame)
        if self.preview or self._qr_window_open:
            cv2.waitKey(1)

    def close(self):
        for camera in (self.camera, self.qr_camera):
            if camera is not None:
                camera.close()
        if self.preview or self._qr_window_open:
            cv2.destroyAllWindows()
        self._qr_window_open = False


def parse_args(argv):
    parser = argparse.ArgumentParser(description='物块、黑环和二维码识别')
    parser.add_argument('--config', help='视觉配置路径；默认 rcs_vision.yaml')
    return parser.parse_args(remove_ros_args(argv)[1:])


def main(args=None):
    argv = sys.argv if args is None else ['rcs_vision', *args]
    options = parse_args(argv)
    config = load_vision_config(options.config)
    rclpy.init(args=argv)
    node = None
    try:
        node = RcsVision(config=config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
