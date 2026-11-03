# 工创原显示渲染器；由独立 ROS 节点更新，不参与控制。
"""Full-Screen Task Code & Gripper Statistics Display Component (12 Digits, 4 Groups).

Layout:
  - Top 3/4: 12-digit task code in 4 groups (e.g. [ 156 ] + [ 123 ] + [ 516 ] + [ 231 ])
    Before QR code is detected, numbers are hidden and replaced with placeholders [ --- ].
  - Bottom 1/4: Gripper stats: 抓取：X   放置：Y (Pick: X, Place: Y)
  - Full-screen auto-adaptation: Scales fonts and paddings dynamically to fit any display.
"""

import ctypes
import glob
import os
import sys
import threading
from typing import List, Tuple

os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def detect_screen_resolution() -> Tuple[int, int]:
    """Auto-detect current monitor resolution across Linux / X11 / Framebuffer."""
    try:
        x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        d = x11.XOpenDisplay(None)
        if d:
            d_ptr = ctypes.c_void_p(d)
            s = x11.XDefaultScreen(d_ptr)
            w = x11.XDisplayWidth(d_ptr, s)
            h = x11.XDisplayHeight(d_ptr, s)
            x11.XCloseDisplay(d_ptr)
            if w > 0 and h > 0:
                return w, h
    except Exception:
        pass

    try:
        fb_path = "/sys/class/graphics/fb0/virtual_size"
        if os.path.exists(fb_path):
            with open(fb_path, "r") as f:
                parts = f.read().strip().split(",")
                if len(parts) == 2:
                    return int(parts[0]), int(parts[1])
    except Exception:
        pass

    return 1920, 1080


def find_chinese_font() -> str | None:
    """Find a system TrueType font supporting Chinese characters."""
    preferred = [
        "/usr/share/fonts/TTF/LXGWWenKai-Regular.ttf",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Bold.ttc",
        "/usr/share/fonts/wps-fonts/timesbd.ttf",
    ]
    for p in preferred:
        if os.path.exists(p):
            return p

    for p in glob.glob("/usr/share/fonts/**/*.ttf", recursive=True):
        try:
            f = ImageFont.truetype(p, 24)
            img = Image.new("L", (40, 40))
            d = ImageDraw.Draw(img)
            bbox = d.textbbox((0, 0), "抓取", font=f)
            if bbox[2] > bbox[0] and bbox[3] > bbox[1]:
                return p
        except Exception:
            continue

    return None


class TaskDisplay:
    """Thread-safe Full-screen Display Component for 12-Digit 4-Group Task Code & Gripper Status."""

    GROUP_LABELS = ["第一批物料颜色", "第一批放置环号", "第二批物料颜色", "第二批放置环号"]

    def __init__(self,
                 width: int | None = None,
                 height: int | None = None,
                 fullscreen: bool = True,
                 show_group_labels: bool = True,
                 window_name: str = "Robot Task & Gripper Monitor"):
        self.lock = threading.Lock()
        self.running = False
        self.thread: threading.Thread | None = None
        self.window_name = window_name
        self.fullscreen = fullscreen
        self.show_group_labels = show_group_labels

        detected_w, detected_h = (width, height) if width and height else detect_screen_resolution()
        self.width = width or detected_w
        self.height = height or detected_h

        self.top_h = int(self.height * 0.75)
        self.bot_h = self.height - self.top_h

        # Data states: task_code is None until QR is detected
        self.task_code: str | None = None
        self.pick_count: int = 0
        self.place_count: int = 0

        # Colors (RGB format for PIL)
        self.bg_color = (15, 23, 42)          # Deep Navy Slate (#0F172A)
        self.card_bg = (30, 41, 59)           # Card Slate (#1E293B)
        self.card_border = (56, 189, 248)     # Vivid Cyan Border (#38BDF8)
        self.card_border_waiting = (71, 85, 105) # Muted border when waiting
        self.task_color = (56, 189, 248)      # Vivid Cyan Digits
        self.placeholder_color = (100, 116, 139) # Grey for '---'
        self.plus_color = (148, 163, 184)     # Muted Slate Grey for '+'
        self.plus_color_waiting = (71, 85, 105)
        self.label_color = (203, 213, 225)    # Light Slate Text
        self.sub_label_color = (148, 163, 184)# Sub-label Grey
        self.pick_color = (74, 222, 128)      # Emerald Green for pick (#4ADE80)
        self.place_color = (251, 146, 60)     # Amber Orange for place (#FB923C)
        self.divider_color = (51, 65, 85)     # Divider Line
        self.warn_color = (251, 191, 36)      # Amber yellow for waiting status

        self.font_path = find_chinese_font()

    def set_task_code(self, code: str | None):
        """Update the task code string. Pass None or empty string to hide numbers."""
        with self.lock:
            if code is None or not str(code).strip():
                self.task_code = None
            else:
                self.task_code = str(code).strip()

    def set_counts(self, pick: int | None = None, place: int | None = None):
        """Update pick and/or place counts directly."""
        with self.lock:
            if pick is not None:
                self.pick_count = max(0, int(pick))
            if place is not None:
                self.place_count = max(0, int(place))

    def increment_pick(self, delta: int = 1):
        """Increment (or decrement) successful pick count."""
        with self.lock:
            self.pick_count = max(0, self.pick_count + delta)

    def increment_place(self, delta: int = 1):
        """Increment (or decrement) successful place count."""
        with self.lock:
            self.place_count = max(0, self.place_count + delta)

    def _parse_groups(self, task_str: str | None) -> Tuple[List[str], bool]:
        """Parse task code into 4 groups. Returns (groups, is_valid)."""
        if not task_str:
            return ["---", "---", "---", "---"], False

        clean = task_str.strip()
        if "+" in clean:
            parts = [p.strip() for p in clean.split("+") if p.strip()]
            if len(parts) == 4:
                return parts, True
            elif parts:
                return parts, True

        digits = "".join(filter(str.isdigit, clean))
        if len(digits) >= 12:
            return [digits[0:3], digits[3:6], digits[6:9], digits[9:12]], True
        elif len(digits) >= 3:
            return [digits[i:i + 3] for i in range(0, len(digits), 3)], True

        return [clean], True

    def _get_font(self, size: int) -> ImageFont.FreeTypeFont:
        if self.font_path:
            return ImageFont.truetype(self.font_path, size)
        return ImageFont.load_default()

    def render_frame(self) -> np.ndarray:
        """Render the complete UI frame as a BGR numpy array for OpenCV."""
        with self.lock:
            raw_task = self.task_code
            pick_cnt = self.pick_count
            place_cnt = self.place_count

        W, H = self.width, self.height
        top_h = self.top_h
        bot_h = self.bot_h

        img = Image.new("RGB", (W, H), self.bg_color)
        draw = ImageDraw.Draw(img)

        # ── 1. Top 3/4 Area: 12-Digit 4-Group Task Code ───────────────────────
        groups, is_valid = self._parse_groups(raw_task)
        num_groups = len(groups)

        # Header Title
        title_font_size = max(16, int(H * 0.03))
        font_title = self._get_font(title_font_size)
        title_text = "任务编码 (TASK CODE - 4 GROUPS)" if is_valid else "任务编码 (TASK CODE - WAITING FOR QR)"
        draw.text((W * 0.03, H * 0.03), title_text, fill=self.plus_color, font=font_title)

        # Waiting prompt text if QR code not detected yet
        card_border_color = self.card_border if is_valid else self.card_border_waiting
        digit_text_color = self.task_color if is_valid else self.placeholder_color
        plus_text_color = self.plus_color if is_valid else self.plus_color_waiting

        target_digit_size = int(min(W * 0.08, top_h * 0.28))
        font_digits = self._get_font(target_digit_size)
        font_plus = self._get_font(int(target_digit_size * 0.80))
        font_sub = self._get_font(max(14, int(target_digit_size * 0.20)))

        sample_bbox = font_digits.getbbox("888")
        gw_sample = sample_bbox[2] - sample_bbox[0]
        gh_sample = sample_bbox[3] - sample_bbox[1]

        card_pad_x = int(target_digit_size * 0.16)
        card_w = gw_sample + card_pad_x * 2
        card_h = gh_sample + int(target_digit_size * 0.35)

        plus_bbox = font_plus.getbbox("+")
        plus_w = plus_bbox[2] - plus_bbox[0]
        plus_h = plus_bbox[3] - plus_bbox[1]
        spacing = int(target_digit_size * 0.22)

        total_w = num_groups * card_w + (num_groups - 1) * (plus_w + spacing * 2)

        if total_w > W * 0.94:
            scale = (W * 0.94) / total_w
            target_digit_size = max(24, int(target_digit_size * scale))
            font_digits = self._get_font(target_digit_size)
            font_plus = self._get_font(int(target_digit_size * 0.80))
            font_sub = self._get_font(max(12, int(target_digit_size * 0.20)))

            sample_bbox = font_digits.getbbox("888")
            gw_sample = sample_bbox[2] - sample_bbox[0]
            gh_sample = sample_bbox[3] - sample_bbox[1]

            card_pad_x = int(target_digit_size * 0.16)
            card_w = gw_sample + card_pad_x * 2
            card_h = gh_sample + int(target_digit_size * 0.35)

            plus_bbox = font_plus.getbbox("+")
            plus_w = plus_bbox[2] - plus_bbox[0]
            plus_h = plus_bbox[3] - plus_bbox[1]
            spacing = int(target_digit_size * 0.22)

            total_w = num_groups * card_w + (num_groups - 1) * (plus_w + spacing * 2)

        start_x = (W - total_w) // 2
        mid_y = int(top_h * 0.50)
        card_y = mid_y - card_h // 2

        # Draw "等待扫描二维码中..." prompt above cards if QR not yet detected
        if not is_valid:
            font_prompt = self._get_font(max(18, int(H * 0.032)))
            prompt_str = "等待扫描二维码中..."
            p_bbox = font_prompt.getbbox(prompt_str)
            pw = p_bbox[2] - p_bbox[0]
            draw.text(((W - pw) // 2, card_y - int(H * 0.055)), prompt_str, fill=self.warn_color, font=font_prompt)

        cur_x = start_x
        for i, g in enumerate(groups):
            bbox = font_digits.getbbox(g)
            w_g = bbox[2] - bbox[0]

            # 1. Rounded Card Background
            draw.rounded_rectangle(
                [(cur_x, card_y), (cur_x + card_w, card_y + card_h)],
                radius=max(6, int(card_h * 0.12)),
                fill=self.card_bg,
                outline=card_border_color,
                width=max(2, int(target_digit_size * 0.02))
            )

            # 2. Centered Text
            tx = cur_x + (card_w - w_g) // 2 - bbox[0]
            ty = card_y + (card_h - gh_sample) // 2 - bbox[1]
            draw.text((tx, ty), g, fill=digit_text_color, font=font_digits)

            # 3. Sub-label annotation
            if self.show_group_labels and i < len(self.GROUP_LABELS):
                label_text = self.GROUP_LABELS[i]
                s_bbox = font_sub.getbbox(label_text)
                sw = s_bbox[2] - s_bbox[0]
                label_y = card_y + card_h + int(H * 0.018)
                draw.text((cur_x + (card_w - sw) // 2, label_y), label_text, fill=self.sub_label_color, font=font_sub)

            cur_x += card_w

            # 4. Connector '+'
            if i < num_groups - 1:
                cur_x += spacing
                px = cur_x - plus_bbox[0]
                py = mid_y - plus_h // 2 - plus_bbox[1]
                draw.text((px, py), "+", fill=plus_text_color, font=font_plus)
                cur_x += plus_w + spacing

        # ── 2. Divider Line ───────────────────────────────────────────────────
        draw.line([(W * 0.02, top_h), (W * 0.98, top_h)], fill=self.divider_color, width=3)

        # ── 3. Bottom 1/4 Area: 抓取：X   放置：Y ─────────────────────────────
        draw.rectangle([(W * 0.015, top_h + 10), (W * 0.985, H - 12)], fill=self.card_bg)

        stat_font_size = int(min(W * 0.065, bot_h * 0.42))
        font_stat = self._get_font(stat_font_size)

        segments = [
            ("抓取：", self.label_color),
            (f" {pick_cnt} ", self.pick_color),
            ("          放置：", self.label_color),
            (f" {place_cnt} ", self.place_color),
        ]

        total_stat_w = sum(font_stat.getbbox(text)[2] - font_stat.getbbox(text)[0] for text, _ in segments)
        stat_x = (W - total_stat_w) // 2
        stat_y = top_h + (bot_h - stat_font_size) // 2

        cur_sx = stat_x
        for text, color in segments:
            bbox = font_stat.getbbox(text)
            draw.text((cur_sx, stat_y), text, fill=color, font=font_stat)
            cur_sx += bbox[2] - bbox[0]

        rgb_arr = np.array(img)
        return cv2.cvtColor(rgb_arr, cv2.COLOR_RGB2BGR)

    def run(self):
        """Run the display window loop (blocking)."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if self.fullscreen:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.resizeWindow(self.window_name, self.width, self.height)

        self.running = True
        print(f"[Display] Window active: {self.width}x{self.height} (Fullscreen={self.fullscreen})")

        while self.running:
            frame = self.render_frame()
            cv2.imshow(self.window_name, frame)

            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break
            elif key == ord("f"):
                self.fullscreen = not self.fullscreen
                prop = cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, prop)
                if not self.fullscreen:
                    cv2.resizeWindow(self.window_name, self.width, self.height)
            elif key in (ord("1"), ord("+"), ord("=")):
                self.increment_pick(1)
            elif key in (ord("2"), ord("]")):
                self.increment_place(1)
            elif key in (ord("3"), ord("-"), ord("_")):
                self.increment_pick(-1)
            elif key in (ord("4"), ord("[")):
                self.increment_place(-1)
            elif key == ord("t"):
                demo_codes = [
                    "156+123+516+231",
                    "123+231+312+123",
                    "642+135+246+531",
                    None,
                ]
                cur_idx = demo_codes.index(self.task_code) if self.task_code in demo_codes else -1
                self.set_task_code(demo_codes[(cur_idx + 1) % len(demo_codes)])

        self.running = False
        cv2.destroyWindow(self.window_name)
        print("[Display] Window closed.")

    def start(self):
        """Start display loop in a background thread."""
        if self.running:
            return
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop display loop."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
