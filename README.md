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
│   ├── rcs_ptl/           #   通信协议（RDLC 封帧、拆帧、校验）
│   └── rcs_serial/        #   串口收发驱动（pyserial + serial_bridge 多话题桥接）
│
└── rcs_tools/             # ── 工具层：辅助开发/调试/仿真（不参与运行）
    ├── rcs_calib/         #   标定（视觉-场地外参/相机内参）
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
| `rcs_ptl` | RDLC 帧编解码与校验 | `rdlc.h` / `RCS_Ptl.h` |
| `rcs_serial` | 串口收发 | `rcs_uart.h` |

## 决策层分工

`rcs_fsm` 提供 Lite 风格内核；`rcs_task` 注册状态、实现比赛决策；`rcs_action` 实现非阻塞功能函数。BT 暂不引入。已有模拟动作示例与 ROS 定时器入口，见 [框架说明](src/rcs_decision/FRAMEWORK.md)。

## 数据流

```text
rcs_task → rcs_action → ROS 业务消息 → rcs_serial 桥接
                                      ↓
                               rcs_msgs 编解码 payload
                                      ↓
                               rcs_ptl RDLC 帧编解码
                                      ↓
                               串口驱动 ↔ 下位机
```

动作反馈沿反向链路返回，按请求编号更新动作状态，由任务决定下一状态。业务格式见 [业务协议](src/rcs_comm/rcs_msgs/BUSINESS.md)。

## 依赖安装

> ROS2 环境由**鱼香ROS 一键安装**搭建：
> ```bash
> wget http://fishros.com/install -O fishros && . fishros
> ```
> 已含：ROS2 Desktop 全功能版（rviz2/tf2/常用消息）、rosdepc、colcon 等工具、环境变量自动配置、系统源切换。

| 依赖 | 用途 | 安装 |
|---|---|---|
| ROS2 Desktop + 工具链 | 节点/话题/服务/消息、RViz、TF、colcon、rosdepc | 一键安装（见上） |
| pyserial | 上位机串口收发（rcs_serial） | `pip install pyserial` |
| Navigation2 | 导航（rcs_navigation） | `sudo apt install ros-humble-navigation2` |
| 视觉库（OpenCV/PyTorch/YOLO） | 目标检测（rcs_vision） | 按算法选装 |
| 点云库（PCL/livox_ros_driver2） | 定位/导航（用雷达时） | 按需安装 |

## 构建与运行

```bash
cd ~/RCS_Robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 run rcs_serial rdlc_serial_test --port /dev/ttyUSB0   # 串口回环测试
ros2 run rcs_serial serial_bridge --ros-args -p port:=/dev/ttyUSB0    # 多话题桥接节点
```

> 串口访问需加入 `dialout` 组：`sudo usermod -aG dialout $USER` 后重新登录。
> 多话题桥接协议约定见 `src/rcs_comm/rcs_serial/BRIDGE.md`。