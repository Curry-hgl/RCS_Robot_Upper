"""多话题串口桥接节点。

职责：
  - 订阅多个业务话题，将消息编码为带类型 ID 的 payload，发往 MCU。
  - 从串口接收 MCU 回帧，按类型 ID 分发解析，发布到对应话题。
  - 单一发送队列 + 工作线程，保证多话题并发时发送不拥挤、优先级可控。

字节约定（串口 payload 首字节固定为消息类型 ID）：
    payload = [MSGTYPE(1字节)] + [各消息自己的字节序列]
"""

from __future__ import annotations

import queue
import struct
import threading
from dataclasses import dataclass
from typing import Callable

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcs_msgs.msg import ChassisCmd, VisionData

from .config import SerialConfig
from .rdlc_link import RdlcSerialLink

# ---- 消息类型 ID（MCU 端必须一致）----
MSG_VISION = 0x01   # 视觉坐标
MSG_CHASSIS = 0x02  # 底盘指令
MSG_TASK = 0x03     # 空占位，供未来扩展

# 底盘模式/优先级取值
CHASSIS_MODE_STOP = 0x00
CHASSIS_MODE_VEL = 0x01

# 视觉目标类型取值
OBJECT_UNKNOWN = 0x00
OBJECT_BLOCK = 0x01
OBJECT_CYLINDER = 0x02


# ---------- 编码器：ROS 消息 -> payload 字节 ----------

def encode_vision(msg: VisionData) -> bytes:
    """视觉坐标：id1 + 类型1字节 + 坐标3xfloat(小端)。"""
    obj_type = OBJECT_BLOCK  # 示例：这里应映射 object_id -> 类型
    return struct.pack("<Bff f", obj_type, msg.position_x, msg.position_y, msg.position_z)


def encode_chassis(msg: ChassisCmd) -> bytes:
    """底盘指令：mode1 + vx/vy/wz 3xfloat + priority1。"""
    return struct.pack("<BfffB", msg.mode, msg.vx, msg.vy, msg.wz, msg.priority)


@dataclass
class TopicRoute:
    """把"订阅什么话题 + 用什么 ID + 怎么编码"绑定为一组。"""

    topic: str
    msg_type: type
    msg_id: int
    encoder: Callable[[object], bytes]
    priority: int = 0  # 越小越优先发送


class SerialBridge(Node):
    def __init__(self, config: SerialConfig | None = None) -> None:
        super().__init__("serial_bridge")

        # 串口端口可从 ROS2 参数读取：--ros-args -p port:=/dev/ttyUSB0
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        self.link = RdlcSerialLink(config or SerialConfig(port=port))
        self._tx_queue: queue.PriorityQueue = queue.PriorityQueue(maxsize=256)

        # 工作线程：既发送待发帧，也持续接收并分发 MCU 回帧
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

        # 注册一个话题 -> 编码路由（可继续往里加）
        routes = [
            TopicRoute("/vision/object", VisionData, MSG_VISION, encode_vision, priority=1),
            TopicRoute("/cmd/chassis", ChassisCmd, MSG_CHASSIS, encode_chassis, priority=0),
        ]
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5, history=HistoryPolicy.KEEP_LAST,
        )
        for route in routes:
            self.create_subscription(
                route.msg_type, route.topic,
                self._make_send_callback(route), qos)

        # 接收方向的反向路由表：ID -> (话题, 解码后的 msg 字段)
        self._rx_handlers = {
            MSG_VISION: ("/vision/ack", self._handle_vision_ack),
            MSG_CHASSIS: ("/cmd/ack", self._handle_chassis_ack),
        }
        self.get_logger().info("serial_bridge 就绪，已注册 %d 个发送话题", len(routes))

    # ---------- 发送：回调只入队，工作线程真正写串口 ----------

    def _make_send_callback(self, route: TopicRoute) -> Callable:
        def cb(msg) -> None:
            try:
                payload = route.encoder(msg)
            except Exception as exc:
                self.get_logger().error("编码 %s 失败: %s", route.topic, exc)
                return
            frame_mark = bytes((route.msg_id,)) + payload
            try:
                # (优先级, 递增序号) 保证 PriorityQueue 不比较 msg 本身
                self._tx_queue.put_nowait((route.priority, self._seq(), frame_mark))
            except queue.Full:
                self.get_logger().warn("发送队列已满，丢弃一条 %s", route.topic)
        return cb

    _seq_count = 0

    def _seq(self) -> int:
        self.__class__._seq_count += 1
        return self.__class__._seq_count

    def _run(self) -> None:
        """工作线程：优先级发送 + 持续接收分发。"""
        import time
        while rclpy.ok():
            if not self.link.transport.is_open:
                time.sleep(0.2)  # 串口未打开，稍后再试
                continue
            try:
                # 发送：取一条待发帧
                _, _, frame_mark = self._tx_queue.get(timeout=0.05)
            except queue.Empty:
                frame_mark = None
            if frame_mark is not None:
                try:
                    self.link.send(frame_mark, dst=0x02)
                except Exception as exc:
                    self.get_logger().error("串口发送失败: %s", exc)
            # 接收：把积压的 MCU 回帧全部收走并分发
            self._drain_replies()

    def _drain_replies(self) -> None:
        """非阻塞收取所有待处理帧并按其类型 ID 分发。"""
        while True:
            try:
                frame = self.link.receive(timeout=0.0)
            except queue.Empty:
                return  # 没有待收帧了
            except Exception as exc:
                self.get_logger().error("串口接收失败: %s", exc)
                return
            msg_id = frame.payload[0] if frame.payload else MSG_TASK
            handler = self._rx_handlers.get(msg_id)
            if handler:
                recv_topic, fn = handler
                # RdlcFrame 冻结不可改，去掉类型 ID 后送处理函数
                fn(recv_topic, frame.payload[1:])
            else:
                self.get_logger().warn("未知消息类型 ID: 0x%02X", msg_id)

    def _handle_vision_ack(self, topic: str, payload: bytes) -> None:
        pass  # 预留：解析 payload 并发布到 topic

    def _handle_chassis_ack(self, topic: str, payload: bytes) -> None:
        pass  # 预留：解析 payload 并发布到 topic

    def open(self) -> None:
        self.link.open()
        self.get_logger().info("串口已打开: %s", self.link.config.port)

    def close(self) -> None:
        self.link.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        node.open()
        rclpy.spin(node)
    finally:
        node.close()
        rclpy.shutdown()