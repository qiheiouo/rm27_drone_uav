# 故障注入与安全回归

## 目的与边界

`simulation/fault_injection.*` 是仅供主机仿真使用的确定性故障注入器。它在正常传感器帧生成后、共享 `NavRuntime` 消费前修改输入，不进入 `nav_core` 或 STM32 固件，因此不会在实机代码中留下测试分支。

注入器使用固定容量数组，最多保存 8 条规则，无堆分配。每条规则声明输入源、故障模式、起止时间、时间偏移和可选最大执行次数；运行状态记录规则命中次数和调度时间累计偏移，可用于判定场景是否真正执行过故障。

## 支持的故障

| 模式 | 作用 | 典型验证点 |
|---|---|---|
| `DROPOUT` | 将指定输入在窗口内置为不可用 | 估计器或跟踪器退化后恢复 |
| `FREEZE_TIMESTAMP` | 保持首次命中的时间戳 | 重复样本拒绝、短时 coast、重新捕获 |
| `DELAY_TIMESTAMP` | 持续减去固定延迟 | 过期样本丢弃、基座重新搜索 |
| `ROLLBACK_TIMESTAMP` | 对指定帧回退时间戳 | 乱序检测和必需输入安全拒绝 |
| `NONFINITE` | 注入 NaN | 数据边界拦截，控制输出保持有限 |
| `CYCLE_OVERRUN` | 累加主循环唤醒延迟，同时平移可用样本时钟 | 连续 overrun、watchdog 锁存和紧急降落 |

调度超时会同时移动运行时和当帧可用传感器的时间戳。这表示任务线程晚唤醒但传感器仍在持续采样，可单独验证调度 watchdog，而不会把同一场景混成“所有传感器都过期”。累计偏移在故障窗口结束后保留，保证时间基准不会倒退。

## 内置场景与通过条件

| 场景 | 注入 | 必须观察到的结果 |
|---|---|---|
| `flow-dropout` | 0.9 秒光流丢包 | 估计器进入降级/恢复路径，最终完成任务 |
| `target-freeze` | 目标时间戳冻结 | TARGET 重复掩码、目标暂不可用，最终重捕获并完成任务 |
| `home-delay` | 基座观测延迟 400 ms | HOME 过期掩码、返回基座搜索，重捕获后完成任务 |
| `nan-target` | 目标尺寸为 NaN | TARGET 无效掩码，异常观测不进入制导，最终完成任务 |
| `imu-stale` | IMU 延迟 100 ms | 本周期被拒绝、解锁标志清除、控制输出归零 |
| `imu-duplicate` | IMU 时间戳冻结 | 重复 IMU 被拒绝、控制输出归零 |
| `imu-rollback` | IMU 时间戳回退 50 ms | 乱序 IMU 被拒绝、控制输出归零 |
| `watchdog-overrun` | 连续两个周期各增加 60 ms | overrun 与 watchdog 事件均出现，状态机完成紧急降落 |

所有有效运行周期还会检查导航状态、制导和控制输出是否为有限值。CTest 使用 `FAULT_REGRESSION_SUCCESS` 作为通过标志；仅触发故障但没有命中声明的健康掩码和最终安全动作会失败。

运行示例：

```powershell
.\build\mission_sim.exe --scenario flow-dropout
.\build\mission_sim.exe --scenario imu-rollback
.\build\mission_sim.exe --scenario watchdog-overrun
ctest --test-dir build -R "fault_" --output-on-failure
```

## 扩展规则

新场景应在 `scenario_apply_kind` 中添加规则和明确预期，不应只检查进程退出码。可恢复故障应证明发生过退化且最终完成任务；必需输入损坏应证明运行时拒绝、撤销解锁并输出零控制；不可恢复的运行故障应证明安全状态机完成紧急降落。时间窗必须落在对应传感器实际参与任务的阶段，避免“注入了但没有影响消费者”的空测试。

该工具验证的是软件失效响应，不代替实物上的断线、总线拥塞、CPU 负载、时钟同步和电机安全测试。有硬件后应把台架实测故障日志转换为回放数据，并用同一健康掩码与安全结果做对照。
