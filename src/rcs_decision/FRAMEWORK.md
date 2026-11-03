# Lite 状态机框架

- `rcs_fsm`：纯 Python 内核，无 ROS、业务消息或动作依赖。
- `rcs_action`：非阻塞动作接口及模拟实现，无 FSM 依赖。
- `rcs_task`：组装内核与动作，注册状态，决定跳转；提供 ROS 定时器适配。
- `rcs_bt`：暂不使用。

参考 `/home/ikun/refference/RoboCup-main/USER/FSM_Lite.c/h` 的周期执行方式重新实现；未引入 STM32/RTOS 或比赛逻辑。

## 调度约定

每次 `Machine.tick()` 最多调用一个状态函数。函数返回 `Event.FINISH` 时，当前周期仍保留原状态，任务可以修改 `set_next()`，下一周期才跳转并执行新状态。`jump()` 也安排在下一周期生效，并优先于普通完成跳转。

`NONE` 表示继续执行；`FAILED`、`PAUSE` 交由任务决策，不自动切换或停止调度。业务可以返回自己的可哈希事件值。缺少状态或后继配置会显式报错。内核异常向上传播，真实任务应定义故障处理策略。

内核由单一线程驱动；ROS 回调与定时器应串行执行，或通过队列将外部事件交给任务线程。

## 替换示例

`rcs_task/example.py` 展示：等待启动 → 执行动作 → 完成，失败转入 ERROR。

在 `rcs_action` 实现 `Action` 接口：

1. `start()`：发起一次新动作，真实实现通过业务消息接口发送指令，记录请求编号。
2. `poll()`：立即返回进行中、成功、失败或已取消，不阻塞、不重复启动。
3. `cancel()`：取消执行；真实硬件需处理取消确认及旧反馈隔离。

把实例传给 `ExampleTask(action)`，或替换任务中的状态注册及决策即可。状态函数不直接读写串口。通信职责继续按 `rcs_msgs`（业务定义/注册/编解码）→ `rcs_ptl`（RDLC）→ `rcs_serial`（收发）划分；业务协议与真实通信运行方式见 [业务协议](../rcs_comm/rcs_msgs/BUSINESS.md)。

`reset()` 会调用已启动动作的取消接口，再重置任务。当前示例模拟取消立即完成；真实动作需要在确认停止后允许重新启动，并自行实现超时处理。

## 运行

```bash
colcon build --symlink-install --packages-select rcs_fsm rcs_action rcs_task
source install/setup.bash
ros2 run rcs_task fsm_example
ros2 run rcs_task task_example --ros-args -p tick_period:=0.05
```

第一条示例不需要硬件，打印流程后退出。ROS 示例每 50ms 驱动一次，自动启动模拟动作，完成后保持 DONE，Ctrl+C 退出。

纯内核与模拟任务测试（无需 ROS）：

```bash
PYTHONPATH=src/rcs_decision/rcs_fsm:src/rcs_decision/rcs_action:src/rcs_decision/rcs_task python3 -m unittest discover -s src/rcs_decision/rcs_task/test -p test_framework.py -v
```
