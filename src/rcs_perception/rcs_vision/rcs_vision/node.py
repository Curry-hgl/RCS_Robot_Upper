"""工创视觉节点：只采集、识别和发布，不持有串口或比赛状态机。"""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcs_msgs.msg import GcsDetection, GcsDetections, GcsTaskCode, GcsMode, GcsStatus
from .camera import Camera
from .detection import detect_blocks, detect_rings
from .display import draw_blocks, draw_rings
from .qr_code import QrDetector, parse_task_code


class GcsVision(Node):
    def __init__(self):
        super().__init__("gcs_vision")
        def param(name, default):
            return self.declare_parameter(name, default).value
        source = str(param("block_camera", "/dev/video20"))
        qr_source = str(param("qr_camera", "0"))
        width, height = param("width", 640), param("height", 480)
        self.preview = param("preview", False)
        self.mode = 0
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
        self.pub = self.create_publisher(GcsDetections, "/gcs/detections", 10)
        self.qr_pub = self.create_publisher(GcsTaskCode, "/gcs/qr_candidate", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_sub = self.create_subscription(GcsMode, "/gcs/detection_mode", self._mode, qos)
        self.status_sub = self.create_subscription(GcsStatus, "/gcs/status", self._status, qos)
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
                # 空候选也发布，使任务能够重置“连续三帧”的确认计数。
                self.qr_pub.publish(GcsTaskCode(raw=candidate))
        if frame is None:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        objects = (detect_blocks(hsv, **self.block_args) if self.mode == 0
                   else detect_rings(frame, hsv, **self.ring_args))
        msg = GcsDetections(mode=self.mode)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gcs_camera_pixels"
        msg.objects = [GcsDetection(color_id=o.color_id, cx=float(o.cx),
                                   cy=float(o.cy), radius=float(o.radius)) for o in objects]
        self.pub.publish(msg)
        if self.preview:
            (draw_blocks if self.mode == 0 else draw_rings)(frame, objects)
            cv2.imshow("GCS detection", frame)
            cv2.waitKey(1)

    def close(self):
        for camera in (self.camera, self.qr_camera):
            if camera is not None:
                camera.close()
        if self.preview:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GcsVision()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
