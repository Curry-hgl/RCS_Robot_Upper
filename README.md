# RCS_Robot

RCS 特色上位机框架，对齐队内下位机模板的 RCS 体系。上位机只设四层：**决策 / 感知 / 通信 / 工具**，系统调度交给 ROS2（节点 = 执行单元）。

## 项目架构

```
RCS_Robot/src/
├── rcs_decision/          # ── 决策层：决定"现在做什么"
│   ├── rcs_task/          #   比赛流程、状态注册与决策、任务参数
│   ├── rcs_fsm/           #   决策机A：状态机（主流程）
│   │   └── rcs_fsm/       #   Lite 风格周期内核（无 ROS 依赖）
│   ├── rcs_bt/            #   行为树预留，当前不使用
│   │   └── rcsbt/         #   自研引擎（blackboard + executor + 节点原语）
│   └── rcs_action/        #   动作原语（直行/抬升/抓取/释放，指令经 comm 下发）
│
├── rcs_perception/        # ── 感知层：知道"我在哪、周围有什么"
│   ├── rcs_vision/        #   视觉识别（对齐下位机 Vision.c 语义）
│   ├── rcs_location/      #   定位（里程计/IMU/LiDAR 融合）
│   └── rcs_navigation/    #   导航（全局规划 + 局部避障，调 nav2）
│
├── rcs_comm/              # ── 通信层：上下位机可靠传数据
│   ├── rcs_msgs/          #   业务消息定义、编号注册、payload 编解码
│   ├── rcs_ptl/           #   通信协议（统一帧接口 + 可替换协议实现）
│   └── rcs_serial/        #   串口收发驱动（pyserial + serial_bridge 多话题桥接）
│
└── rcs_tools/             # ── 工具层：辅助开发/调试/仿真（不参与运行）
    ├── rcs_debug/         #   调试面板（RViz 插件/数据回放/参数热改）
    └── rcs_sim/           #   仿真（下位机模型 + 上位机节点联调）
```

> `rcs_decision` / `rcs_perception` / `rcs_comm` / `rcs_tools` 是**归类目录（非包）**；真正的一级包是带 `package.xml` 的子包，`colcon build` 只构建它们，空目录自动跳过。

## 各层与下位机对应关系

| 上位机包 | 职责 | 对齐下位机 |
|---|---|---|
| `rcs_task` | 比赛流程与决策 | 任务流程 |
| `rcs_fsm` | 主流程状态机 | `FSM_Lite.c` |
| `rcs_bt` | 复杂分支行为树 | `Behavior_Tree.c` |
| `rcs_action` | 动作原语 | `Up_Ctrl.c` / `Ch_Ctrl.c` |
| `rcs_vision` | 视觉识别输出 | `Vision.c` |
| `rcs_ptl` | 统一帧接口，默认使用 RDLC | `rdlc.h` / `RCS_Ptl.h` |
| `rcs_serial` | 串口收发 | `rcs_uart.h` |

## 决策层分工

- `rcs_fsm`：提供 Lite 风格内核。
- `rcs_task`：注册状态、实现比赛决策。
- `rcs_action`：实现非阻塞功能函数。
- BT：暂不引入。
- 任务按 `upper / chassis / vision` 选择，默认只启用 `vision`。

## 数据流

```text
rcs_task → rcs_action → ROS 业务消息 → rcs_serial 桥接
                                      ↓
                               rcs_msgs 编解码 payload
                                      ↓
                               rcs_ptl 帧封装与解析
                                      ↓
                               串口驱动 ↔ 下位机
```

动作反馈沿反向链路返回，按请求编号更新动作状态，由任务决定下一状态。业务文件位于 `rcs_msgs/business/`：

| 文件 | 职责 |
|---|---|
| `rcs_msgs_payload.py` | 业务载荷校验、编码和解码 |
| `rcs_msgs_registry.py` | 业务编号与编解码函数的注册映射 |

| 协议文件 | 职责 |
|---|---|
| `rcs_ptl_frame.py` | 统一帧接口 |
| `rcs_ptl_rdlc.py` | 协议实现，切换协议时替换此文件 |

## 运行

```bash
ros2 run rcs_serial list_ports                            # 列出串口
ros2 run rcs_serial rcs_serial_loopback --port /dev/ttyUSB0   # 串口回环测试
ros2 run rcs_serial serial_bridge --ros-args -p port:=/dev/ttyUSB0    # 多话题桥接节点
```

> 串口访问需加入 `dialout` 组：`sudo usermod -aG dialout $USER` 后重新登录。


### RC 国赛启动

```bash
ros2 run rcs_task rcs_task vision        # 完整视觉流程
ros2 run rcs_task rcs_task upper vision  # 组合任务，共享串口
ros2 run rcs_task rcs_task               # 按总配置启动
```

| 配置 | 用途 |
|---|---|
| [`rcs_task.yaml`](src/rcs_decision/rcs_task/config/rcs_task.yaml) | 默认运行哪些任务 |
| [`rcs_task_vision.yaml`](src/rcs_decision/rcs_task/config/rcs_task_vision.yaml) | 视觉任务的串口、显示、发送周期 |
| [`rcs_vision.yaml`](src/rcs_perception/rcs_vision/config/rcs_vision.yaml) | 相机、识别模式、阈值、二维码及画面预览 |
| [`rcs_task_upper.yaml`](src/rcs_decision/rcs_task/config/rcs_task_upper.yaml) | 上层任务周期、串口 |
| [`rcs_task_chassis.yaml`](src/rcs_decision/rcs_task/config/rcs_task_chassis.yaml) | 底盘任务周期、串口 |

各任务也有独立 launch，例如 `ros2 launch rcs_task rcs_task_vision.launch.py`；总入口为 `rcs_task.launch.py`。

| 任务文件 | 职责 |
|---|---|
| `rcs_task_upper.py` | 上层机构任务框架 |
| `rcs_task_chassis.py` | 底盘指令与任务框架 |
| `rcs_task_vision.py` | 视觉任务 |

当前业务协议占用 `0x01～0x05`，底盘编号改为 `0x20`，通用三维视觉改为 `0x21`。空检测帧会发送数量 0，下位机应据此清除旧目标。

### 无硬件验证

```bash
ros2 run rcs_ptl test             # 全部协议测试
ros2 run rcs_ptl test frame       # 只测默认协议收发
ros2 run rcs_ptl test protocol    # 只测协议切换
ros2 run rcs_task test            # 全部任务测试
ros2 run rcs_task test serial_link  # 只测视觉与模拟串口链路
ros2 run rcs_task test action_link  # 只测动作与反馈链路
ros2 run rcs_vision test          # 视觉测试
```

伪串口测试不需要相机或下位机；真实相机驱动、光照标定、二维码实拍识别效果和下位机固件仍需实机联调。

## 依赖安装

> ROS2 环境由**鱼香ROS 一键安装**搭建：
> ```bash
> wget http://fishros.com/install -O fishros && . fishros
> ```
> 已含：ROS2 Desktop 全功能版（rviz2/tf2/常用消息）、rosdepc、colcon 等工具、环境变量自动配置、系统源切换。

| 依赖 | 用途 | 安装 |
|---|---|---|
| ROS2 Desktop + 工具链 | 节点/话题/服务/消息、RViz、TF、colcon、rosdepc | 一键安装（见上） |
| pytest | `ros2 run` 测试入口的运行依赖 | `sudo apt install python3-pytest` |
| pyserial | 上位机串口收发（rcs_serial） | `pip install pyserial` |
| Navigation2 | 导航（rcs_navigation） | `sudo apt install ros-humble-navigation2` |
| 视觉库（OpenCV/PyTorch/YOLO） | 目标检测（rcs_vision） | 按算法选装 |
| 点云库（PCL/livox_ros_driver2） | 定位/导航（用雷达时） | 按需安装 |
