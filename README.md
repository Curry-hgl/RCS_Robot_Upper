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

`rcs_fsm` 提供 Lite 风格内核；`rcs_task` 注册状态、实现比赛决策；`rcs_action` 实现非阻塞功能函数。BT 暂不引入。提供模拟动作示例与 ROS 定时器入口，运行方法见下文。

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

动作反馈沿反向链路返回，按请求编号更新动作状态，由任务决定下一状态。业务格式和编号注册集中在 `rcs_msgs/business/`。

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

## 工创赛视觉

已移植物块/黑环识别、二维码任务流程和抓放统计，沿用工创消息编号。默认主相机 `/dev/video20`，二维码相机 `0`，分辨率 640×480。

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


### 工创赛启动

构建并 source 后，一键启动视觉、任务和串口桥接：

```bash
ros2 launch rcs_task gcs.launch.py
```

常用配置：

```bash
# 单相机，不扫描二维码
ros2 launch rcs_task gcs.launch.py block_camera:=0 qr_enabled:=false

# 打开检测预览和任务状态大屏，需要桌面环境
ros2 launch rcs_task gcs.launch.py preview:=true monitor:=true fullscreen:=true

# 修改串口、相机和发送周期
ros2 launch rcs_task gcs.launch.py port:=/dev/ttyUSB0 block_camera:=/dev/video20 qr_camera:=0 send_interval:=0.5 qr_send_duration:=5.0

# 使用原 WeChat 二维码后端
ros2 launch rcs_task gcs.launch.py qr_backend:=wechat qr_model_dir:=/实际模型目录
```

默认二维码后端为 OpenCV，无需模型。WeChat 后端需要带 `wechat_qrcode` 的 OpenCV，以及目录中的 `detect.prototxt`、`detect.caffemodel`、`sr.prototxt`、`sr.caffemodel`。

二维码连续三帧确认后立即发送，并默认广播 5 秒；物块/黑环结果默认每 0.5 秒发送一次。`initial_mode:=0` 为物块，`initial_mode:=1` 为黑环，下位机可以覆盖模式。

状态窗口按键：`m` 切换识别模式，`1/3` 增减抓取计数，`2/4` 增减放置计数，`f` 切换全屏，`ESC` 关闭显示。关闭显示不影响任务和通信。

需要分别运行时，在各终端 source 工作空间：

```bash
ros2 run rcs_serial serial_bridge --ros-args -p port:=/dev/ttyUSB0
ros2 run rcs_task gcs_task
ros2 run rcs_vision gcs_vision
ros2 run rcs_debug gcs_monitor
```

视觉半径、圆度、填充率等参数在 `rcs_vision/node.py` 声明，单独运行视觉节点时用 `--ros-args -p 参数名:=值` 配置。

工创协议占用 `0x01～0x05`，底盘编号改为 `0x20`，通用三维视觉改为 `0x21`。工创空检测帧会发送数量 0，下位机应据此清除旧目标。

为兼容原工创程序，RDLC 默认不开启转义；当前非转义解码遇到载荷中的 C0/0C 会影响收帧。两端都支持时，通过 `escape_enabled:=true` 同时启用，不能只改上位机。

### FSM 示例

```bash
# 纯模拟流程，打印结果后退出
ros2 run rcs_task fsm_example

# ROS 定时器驱动的模拟动作
ros2 run rcs_task task_example --ros-args -p tick_period:=0.05

# 真实通信示例：下位机必须实现动作码 1（升降），高度单位米
ros2 run rcs_task task_example --ros-args -p simulate:=false -p target_height:=0.1
```

真实动作示例需要先启动串口桥接，不适用于尚未实现该动作协议的工创旧固件。

### 无硬件验证

安装并 source 后运行：

```bash
python3 -m unittest discover -s src/rcs_decision/rcs_task/test -v
python3 -m unittest discover -s src/rcs_perception/rcs_vision/test -v
python3 -m unittest discover -s src/rcs_comm/rcs_ptl/test -v
python3 src/rcs_comm/rcs_serial/test/integration_gcs_link.py
python3 src/rcs_comm/rcs_serial/test/integration_action_link.py
```

伪串口测试不需要相机或下位机；真实相机驱动、光照标定、二维码实拍识别效果和下位机固件仍需实机联调。
