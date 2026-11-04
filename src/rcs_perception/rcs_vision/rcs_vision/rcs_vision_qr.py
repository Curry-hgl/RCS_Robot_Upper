"""二维码识别与结构解析。默认使用无需模型的 OpenCV，可选择原 WeChat 模型。"""
from pathlib import Path
import cv2
from .rcs_vision_config import TaskCode


class QrDetector:
    def __init__(self, backend="opencv", model_dir="~/.opencv/wechat_qrcode"):
        if backend not in ("opencv", "wechat"):
            raise ValueError("qr_backend must be opencv or wechat")
        self.backend = backend
        if backend == "wechat":
            names = ("detect.prototxt", "detect.caffemodel", "sr.prototxt", "sr.caffemodel")
            paths = [str(Path(model_dir).expanduser() / name) for name in names]
            if not all(Path(p).is_file() for p in paths):
                raise RuntimeError("WeChat 二维码模型缺失，请配置 qr_model_dir")
            self.detector = cv2.wechat_qrcode_WeChatQRCode(*paths)
        else:
            self.detector = cv2.QRCodeDetector()

    def decode(self, frame):
        """统一不同 OpenCV 后端的返回值，仅返回识别到的文本。"""
        if self.backend == "wechat":
            texts, _ = self.detector.detectAndDecode(frame)
            return list(texts)
        text, _, _ = self.detector.detectAndDecode(frame)
        return [text] if text else []


def parse_task_code(text: str) -> TaskCode:
    """Parse task code string like '156+123+516+231' into TaskCode.

    Format: group1_colors + group1_positions + group2_colors + group2_positions
    Each group is a 3-digit number, each digit 1-6.
    """
    tc = TaskCode(raw=text)
    text = text.strip()
    tc.raw = text
    parts = text.split("+")
    if len(parts) != 4:
        return tc
    try:
        nums = [int(p) for p in parts]
        if not all(100 <= n <= 999 for n in nums):
            return tc
        tc.batch1_colors = [int(d) for d in str(nums[0])]
        tc.batch1_positions = [int(d) for d in str(nums[1])]
        tc.batch2_colors = [int(d) for d in str(nums[2])]
        tc.batch2_positions = [int(d) for d in str(nums[3])]
        if not all(1 <= c <= 6 for c in tc.batch1_colors + tc.batch2_colors):
            return tc
        if not all(1 <= p <= 6 for p in tc.batch1_positions + tc.batch2_positions):
            return tc
        tc.valid = True
    except (ValueError, IndexError):
        pass
    return tc
