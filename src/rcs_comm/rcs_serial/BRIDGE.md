# serial_bridge 多话题串口桥接节点

`sercial_bridge` 把 **ROS2 多话题** 与 **单条串口链路** 桥接起来，让上层业务通过话题
收发数据，底层继续复用 `rcs_ptl`（RDLC 编解码）+ `rcs_serial`（串口读写）这套
已有通信栈。

```
视觉/决策节点                 serial_bridge 节点                   MCU
--------------------------------------------------------------
/vision/object ───订阅──►  encode_vision ──┐
/cmd/chassis   ───订阅──► encode_chassis ──┴─► 发送队列 ──► 串口 ──► MCU
                                             ▼
                                RdlcSerialLink.send(payload)
                                                   ▲
MCU ──► 串口 ──► RdlcSerialLink.receive ──► 按 ID 分发 ──► 发话题
```

## １. 字节协议约定（MCU 端必须一致）

每帧载荷的 **首字节固定为"消息类型 ID"**，作用是让 MCU（或本节点接收时）
区分这是哪一路数据。RDLC 只负责把这串字节可靠送达，不关心内容。

```text
payload = [MSGTYPE(1字节)] + [各消息自己的字节序列]
```

| 类型 ID | 名称       | 方向             | 对应话题      | 载荷内容 |
|---------|-----------|------------------|--------------|----------|
| 0x01    | MSG_VISION| 下行(发MCU)       | /vision/object | 类型1 + 坐标 |
| 0x02    | MSG_CHASSIS | 下行(发MCU)    | /cmd/chassis  | mode + 速度 |
| 0x03    | MSG_TASK  | 预留             | -              | -        |

## ２. 如何运行

```bash
cd ~/RCS_Robot && source install/setup.bash
ros2 run rcs_serial serial_bridge --ros-args -p port:=/dev/ttyUSB0
```

> `port` 目前尚未接入 ROS2 参数（仍在 SerialConfig 里），后续可改为
> `--ros-args -p port:=...` 从 yaml 读取。

## ３. 支持的话题与编码格式

### 下行（本节点订阅 → 发往 MCU）

| 话题           | 消息类型      | payload 格式 |
|---------------|--------------|--------------|
| `/vision/object` | `VisionData` | `0x01` + `<B fff`：object_id + position_x/y/z |
| `/cmd/chassis`   | `ChassisCmd` | `0x02` + `<B fff B`：mode + vx/vy/wz + priority |

### 上行（MCU → 本节点 → 发布）

| MCU 类型 ID | 对应接收话题 | 说明 |
|------------|--------------|------|
| 0x01       | `/vision/ack`  | 视觉应答（处理函数待实现） |
| 0x02       | `/cmd/ack`     | 底盘应答（处理函数待实现） |

## ４. 如何新增一个话题

只需在 `bridge.py` 的 `routes` 列表里加一条路由，其余自动接好：

```python
routes = [
    TopicRoute("/vision/object", VisionData, MSG_VISION, encode_vision, priority=1),
    TopicRoute("/cmd/chassis", ChassisCmd, MSG_CHASSIS, encode_chassis, priority=0),
    TopicRoute("/my/topic", MyMsg, MSG_MY, encode_my, priority=1),  # 新增
]
```

同时定义编码函数与接收处理即可，发送队列、优先级、类型区分都复用现成机制。

## ５. 核心设计说明

- **单一发送队列**：所有话题回调只把"待发帧"塞进 `queue.PriorityQueue`，
  由单个工作线程统一写串口，避免多回调并发写导致字节交错。
  - `priority` 越小越先发送（底盘 0 > 视觉 1）。
- **接收分发**：收帧后读 `payload[0]` 查表，按 ID 路由到不同处理函数/话题。
- **QoS**：下行话题用 `BEST_EFFORT + depth5`，适合控制类高频数据，
  丢旧数据优于积压。

## ６. 注意事项

- **带宽**：115200 baud ≈ 11.5 KB/s。高频多话题容易排队积压，注意降频或
  只在目标变化时发送。
- **stderr**：本节点不主动主动发送数据，只有收到话题消息或 MCU 回帧才动作。
- 类型 ID 与字节格式必须与 MCU 端保持一致，改任一端都要同步改动。

## ７. 文件位置

```
rcs_serial/rcs_serial/
├── bridge.py        ← 本节点（多话题桥接）
├── rdlc_link.py     ← 串口 + RDLC 组合链路（底层，无需改）
└── serial_test.py   ← 回环监听测试（echo 模式）
```
