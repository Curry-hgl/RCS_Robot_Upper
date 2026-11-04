# 移植自 gcs_upper/upper_machine；算法与原阈值保持一致。
# 本模块不访问串口、不决定比赛阶段，供视觉节点独立调用。
"""RC 国赛颜色编号、原 HSV 标定阈值及算法数据结构；坐标单位为像素。"""

from dataclasses import dataclass, field

import numpy as np


# ── Color ID mapping ──────────────────────────────────────────────────────────

COLOR_MAP = {
    1: "red",
    2: "yellow",
    3: "blue",
    4: "green",
    5: "black",
    6: "sky-blue",
}

COLOR_NAME_TO_ID = {
    "red": 1, "yellow": 2, "blue": 3, "green": 4, "black": 5, "sky-blue": 6,
}


# ── Block HSV thresholds ─────────────────────────────────────────────────────

COLOR_HSV = {
    "red": {
        "lower": np.array([0, 70, 70]),
        "upper": np.array([10, 255, 255]),
        "bgr": (0, 0, 255),
    },
    "red2": {
        "lower": np.array([156, 70, 70]),
        "upper": np.array([180, 255, 255]),
        "bgr": (0, 0, 255),
    },
    "yellow": {
        "lower": np.array([10, 85, 120]),
        "upper": np.array([34, 175, 255]),
        "bgr": (0, 255, 255),
    },
    "blue": {
        "lower": np.array([98, 179, 0]),
        "upper": np.array([121, 255, 255]),
        "bgr": (255, 0, 0),
    },
    "green": {
        "lower": np.array([58, 73, 0]),
        "upper": np.array([74, 255, 186]),
        "bgr": (0, 255, 0),
    },
    "sky-blue": {
        "lower": np.array([98, 0, 0]),
        "upper": np.array([121, 190, 255]),
        "bgr": (255, 255, 0),
    },
    "black": {
        "lower": np.array([0, 0, 0]),
        "upper": np.array([179, 255, 96]),
        "bgr": (50, 50, 50),
    },
}


# ── Ring HSV thresholds — black ring only ────────────────────────────────────

RING_BAND_INNER = 0.7  # annulus inner fraction

RING_HSV = {
    "black": {
        "lower": np.array([0, 0, 0]),
        "upper": np.array([179, 255, 96]),
        "bgr": (50, 50, 50),
    },
}

MIN_BLOCK_RADIUS = 20


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class BlockInfo:
    color: str
    color_id: int
    cx: int
    cy: int
    radius: int


@dataclass
class RingInfo:
    color: str
    color_id: int
    cx: int
    cy: int
    radius: int


@dataclass
class TaskCode:
    raw: str = ""
    batch1_colors: list[int] = field(default_factory=list)
    batch1_positions: list[int] = field(default_factory=list)
    batch2_colors: list[int] = field(default_factory=list)
    batch2_positions: list[int] = field(default_factory=list)
    valid: bool = False


def load_vision_config(path=None):
    from pathlib import Path
    import yaml
    from ament_index_python.packages import get_package_share_directory

    path = Path(path).expanduser() if path else (
        Path(get_package_share_directory('rcs_vision')) / 'config/rcs_vision.yaml')
    with path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('Vision configuration must be a mapping')
    return config
