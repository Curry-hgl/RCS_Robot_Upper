# 业务协议 v1

业务定义、注册和 payload 编解码位于本包 `business/`。serial 只做 ROS/串口适配，ptl 只做 RDLC。所有多字节字段小端、浮点 IEEE754 float32。

请求 payload（9 字节头）：`类型 u8=0x10 | 版本 u8=1 | 请求编号 u32 | 动作码 u16 | 操作 u8 | 参数 bytes`

反馈 payload（11 字节头）：`类型 u8=0x11 | 版本 u8=1 | 请求编号 u32 | 动作码 u16 | 状态 u8 | 错误码 u16 | 结果 bytes`

- 请求编号非零，执行与取消用同一编号；反馈必须带回原编号和动作码。
- 操作：0 执行，1 取消。
- 状态：0 已接收，1 执行中，2 成功，3 失败，4 已取消；错误码 0 表示无错误。
- 参数/结果最多 116 字节，具体结构按动作码注册，不在驱动中硬编码。
- 下位机按动作码分发；相同请求编号的重复执行不可重复启动动作，应返回已有状态；取消需反馈终态。
- 上位机只接受匹配请求编号和动作码的反馈；默认不自动重发，5 秒超时发送取消并报告失败。超时不代表硬件已停止，重新启动前必须确认下位机状态。

## 可替换示例

动作码 1：升降到目标高度；参数为 `<f`，单位米，有限且非负。该动作仅是约定示例，需下位机实现后才能驱动实际机构。无结果参数；成功反馈即可结束动作。

示例请求：编号 1，高度 0.1m：
`10 01 01 00 00 00 01 00 00 cd cc cc 3d`

成功反馈：
`11 01 01 00 00 00 01 00 02 00 00`

原有视觉类型 0x01 和底盘类型 0x02 保留，编解码已移入本包注册表。

注意 RDLC 当前非转义解码会把 payload 中 C0/0C 解释为边界，因此任意二进制参数必须在两端同时启用转义；只开启上位机无效。可通过 `escape_enabled:=true` 配置桥接。

## ROS 链路

`rcs_task → rcs_action.RosAction → /action/command → serial_bridge → rcs_msgs.business → RDLC → 下位机`

反馈原路返回 `/action/feedback`，更新动作状态，再由任务决定下一状态。

构建并 source 后，先启动桥接，再显式启动真实通信示例：

```bash
ros2 run rcs_serial serial_bridge --ros-args -p port:=/dev/ttyUSB0 -p escape_enabled:=true
ros2 run rcs_task task_example --ros-args -p simulate:=false -p target_height:=0.1
```

默认 `simulate:=true`，不会向硬件发送动作。任务 ROS 回调采用单线程执行器；启动真实示例时会自动请求一次动作。示例等待命令订阅和反馈发布端被发现后才推进流程。
