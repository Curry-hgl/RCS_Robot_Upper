"""后台采集最新帧；ROS 定时器不等待相机 read，不重复消费同一帧。"""
import threading
import cv2


class Camera:
    def __init__(self, source, width=640, height=480):
        self.source = int(source) if str(source).isdigit() else str(source)
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f"无法打开相机: {source}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._frame = None
        self._sequence = 0
        self._consumed = 0
        self._thread = threading.Thread(target=self._capture, daemon=True)
        self._thread.start()

    def _capture(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            with self._lock:
                self._frame = frame if ok else None
                self._sequence += 1
            if not ok:
                self._stop.wait(0.02)

    def read(self):
        """返回尚未消费的 BGR 帧；没有新帧或读取失败时返回 None。"""
        with self._lock:
            if self._sequence == self._consumed or self._frame is None:
                return None
            self._consumed = self._sequence
            return self._frame.copy()

    def close(self):
        self._stop.set()
        self.cap.release()
        self._thread.join(timeout=1)
