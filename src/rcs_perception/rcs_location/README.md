# rcs_location

| 入口 | 用途 |
| --- | --- |
| `ros2 run rcs_location rcs_location` | 启动 IMU 驱动与解算 |
| `ros2 run rcs_location rcs_location algorithm` | 仅解算，复用已启动的 IMU |
| `ros2 run rcs_location test` | 轨迹、话题与归零测试 |

- `config/rcs_location.yaml`：驱动开关、配置路径。
- `config/rcs_location_algorithm.yaml`：话题、安装位置、初始化和噪声；修改后重启生效。
- `algorithm/`：解算核心与 ROS 节点；融合层后续接入。
- 查看：`ros2 topic echo /rcs_location/algorithm/imu/state`，默认 cm、deg；速度单位为 cm/s、deg/s。
- `length_unit` 支持 m/cm/mm，`angle_unit` 支持 deg/rad；`state_precision` 默认 2 位小数，修改后重启。
- 标准里程计 `/rcs_location/algorithm/imu/odom` 和 TF 固定使用 m、rad。
- `/rcs_location_algorithm/ready`：初始化完成且输入有效。
- 归零：`ros2 service call /rcs_location_algorithm/reset std_srvs/srv/Trigger '{}'`。

启动和归零时保持静止，自动估计 XY 加速度死区；`acceleration_deadband` 和 `deadband_sigma` 控制阈值，单位固定 m/s²。

`zero_velocity_enabled` 默认开启：低动态持续后清零速度，`motion_confirm_seconds` 过滤退出时的短脉冲；可能误判匀速，可关闭。死区会削弱小幅运动加速度，纯 IMU 仍会漂移。
当前驱动使用主机接收时间戳。加速度正负方向需实车验证，单靠静止重力不能验证运动方向。
