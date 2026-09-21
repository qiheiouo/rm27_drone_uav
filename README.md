# RM_Drone_Nav

面向 RoboMaster 微型无人机的 Plan B 自主导航算法原型。项目以 STM32 级资源约束为设计边界，在主机仿真中打通：起飞、出航、目标搜索与接近、受控接触、撞击后恢复、返航、基座重定位和停靠。

> 当前结论：这是可编译、可回归的 C99 算法原型和 STM32 移植骨架，不是已经完成实机飞行验证的固件。图像检测器、板级驱动、低层姿态控制和硬件参数标定仍需在目标平台完成。

## 已实现能力

| 子系统 | 当前实现 |
|---|---|
| 状态估计 | IMU 姿态/惯性预测、光流 VO 水平修正、ToF 高度融合、视觉绝对修正门限，以及 INIT/DEGRADED/LOST/RECOVERING/RELOCALIZED 等健康状态 |
| 目标与基座跟踪 | 固定内存 alpha-beta 相对状态跟踪、置信度与观测年龄、短时预测、LOST 后重新建轨；基座相对位置和偏航滤波 |
| 任务管理 | 19 个顶层状态，包含目标确认与丢失、受控接触、分阶段恢复、强制返航、基座搜索、精细归航、停靠和紧急降落 |
| 末端制导 | PURSUIT/CLOSING/FINAL_ALIGN/CONTACT_APPROACH/REACQUIRE 分层，速度/加速度限制和移动目标速度前馈 |
| 撞击处理 | 加速度、角速度、速度跳变、姿态跳变融合；布防、去抖、不应期；视觉保持、姿态恢复、速度阻尼、估计器检查和脱离 |
| 轨迹规划 | 固定容量五次多项式轨迹；速度、加速度、时长、工作空间和碰撞后检查；有界时间拉伸；固定内存局部绕行和简单制导兜底 |
| 安全管理 | 软/硬任务截止时间、围栏、估计器质量、轨迹有效性、控制饱和和碰撞风险分级；短时风险去抖 |
| 动态障碍 | 最多 8 个局部/动态障碍，最近接预测、横向避让和紧急爬升输出 |
| 多机扩展 | 固定 48 字节版本化状态帧、CRC、重复/乱序/超时邻机表，最多 4 个他机状态、带时间戳未来轨迹消息、未来冲突检测及按 `agent_id` 确定性让行；默认可关闭 |
| 双 MCU/STM32 对接 | 主机与 STM32 共用 `NavRuntime`；板级只负责输入输出适配；固定内存 FCU 状态/指令契约、速度优先与加速度回退、序号/CRC/超时保护和确定性 UART 断线回归 |
| 多机网络仿真 | 主机侧支持 2/4 个独立 `NavRuntime`，可注入延迟、丢包、乱序和链路失联，用于验证多机冲突与失联回归 |
| 多机失联保护 | 冲突邻机失联时进入保守保持，待链路恢复和状态确认后才解除保护；默认可关闭 |
| 诊断与回放 | 输入过期/重复/乱序检查、连续周期 watchdog、64 条固定内存事件环、确定性 CSV 回放，以及带版本/CRC 的二进制遥测 |
| 故障回归 | 固定内存、确定性的传感器丢包/时间戳异常/非有限值/调度超时注入，并校验恢复、拒绝输出和紧急降落结果 |

算法选择参考了 EGO-Planner、EGO-Swarm、分布式群体轨迹优化、bearing 相对定位、《Swarm of micro flying robots in the wild》及 GCOPTER。项目只提取固定维度轨迹、约束后检查、轨迹时标缩放、带时效的未来状态共享和相对观测恢复等思想；未把 ESDF、ROS、完整 VIO、L-BFGS 或一般非线性优化器直接搬到 MCU。

## 构建与验证

依赖 CMake 3.16+ 和 C99 编译器。Windows MinGW 示例：

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

提交或合并前建议运行统一质量门禁；它还会检查构建产物误提交、本机绝对路径和
编译器警告：

```powershell
cmake -DQUALITY_BUILD_DIR=build -DQUALITY_CONFIG=Debug -P cmake/quality_gate.cmake
```

本机与 Gitee 流水线接入方法见 [持续集成质量门禁](docs/quality_gate.md)。

`build/` 是约定的本机构建目录，`build*/` 均已被 Git 忽略，可在切换机器、生成器或源码路径后安全删除并重新生成。不要复制或提交 CMake 缓存和编译产物。

默认仿真：

```powershell
.\build\mission_sim.exe
.\build\mission_sim.exe --list-scenarios
.\build\mission_sim.exe --scenario target-loss
.\build\mission_sim.exe --scenario local-obstacle
.\build\mission_sim.exe --scenario two-agent-conflict
.\build\swarm_sim.exe --scenario nominal
.\build\swarm_sim.exe --scenario lossy
.\build\swarm_sim.exe --scenario outage
.\build\swarm_sim.exe --scenario guarded-outage
.\build\swarm_sim.exe --scenario four-agent
.\build\mission_sim.exe --scenario flow-dropout
.\build\mission_sim.exe --scenario imu-stale
.\build\mission_sim.exe --scenario watchdog-overrun
.\build\mission_sim.exe --lost-on-impact --seed 17
.\build\mission_sim.exe --hard-impact --seed 23
.\build\nav_replay.exe tests\data\replay_stationary.csv
.\build\nav_replay.exe --verify tests\data\replay_stationary.csv
.\build\mission_sim.exe --telemetry build\mission.bin
.\build\nav_telemetry_dump.exe build\mission.bin
.\build\nav_telemetry_dump.exe --csv build\mission.bin
.\build\fcu_bridge_sim.exe
```

退出码：普通任务中，`0` 为任务完成并停靠，`2` 为紧急降落，`3` 为仿真超时，`4` 表示声明的压力分支没有真正触发，`5` 表示共享运行时拒绝无效输入，`6` 表示运行时产生非有限输出，`7` 表示遥测写入、编码或关闭失败。故障回归场景只有在实际结果、诊断掩码和安全动作均符合声明时才返回 `0`，并输出 `FAULT_REGRESSION_SUCCESS`；因此 `imu-stale` 的“安全拒绝”和 `watchdog-overrun` 的“紧急降落”属于测试通过，而不是任务成功。

当前 CTest 共 82 项：

- 19 个模块级测试（含共享运行时、输入时效、watchdog、事件环、故障注入器、遥测编解码、集群链路、失联保护、确定性网络与 STM32 非阻塞收发、失效轨迹及蜂群安全链路）；
- 1 个双 MCU UART 延迟、损坏和断线回归；
- 2 个 STM32 适配集成测试（FCU 和集群链路）；
- 5 个多运行时网络回归场景（正常链路、有损乱序、链路中断、冲突中断保护和四机容量）；
- 8 个端到端故障回归场景；
- 4 个遥测录制与逐帧校验集成测试（含运行时拒绝时的故障现场保留）；
- 1 个默认闭环测试；
- 1 个确定性日志回放测试；
- 9 个具名压力场景；
- 32 个随机种子、撞击后 LOST、剧烈撞击及真值对照回归。

具名场景包括 `moving-target`、`target-loss`、`impact-degraded`、`impact-lost`、`home-initial-hidden`、`home-loss`、`local-obstacle`、`forced-return` 和 `two-agent-conflict`。场景程序会检查相应异常分支确实被执行，而不只检查最终出现 `MISSION_SUCCESS`。

故障场景包括 `flow-dropout`、`target-freeze`、`home-delay`、`nan-target`、`imu-stale`、`imu-duplicate`、`imu-rollback` 和 `watchdog-overrun`。完整规则、判定标准与扩展方式见[故障注入与安全回归](docs/fault_injection.md)。

遥测格式、带宽和板级队列约束见[结构化遥测](docs/telemetry.md)。遥测保存运行结果，用于诊断；CSV 回放保存运行输入，用于重现。二者不能互相替代。

集群状态帧、邻机生命周期和板级通信约束见[集群链路协议与邻机表](docs/swarm_link.md)。
多运行时、故障网络和四机容量回归见[确定性多机网络仿真](docs/swarm_simulation.md)。
冲突邻机失联时的位置保持、恢复确认和安全优先级见[冲突邻机失联保护](docs/swarm_link_guard.md)。

## 目录边界

```text
core/            MCU 兼容的共享运行时、数学、估计、FSM、制导、规划和安全算法
perception/      相机模型、光流前端、目标/基座跟踪
flight/          导航到低层飞控的控制接口
swarm/           可关闭的多机状态、轨迹消息和冲突处理
platform/host/   主机闭环仿真入口
platform/stm32/  STM32 板级接口与 NavApp 装配骨架
simulation/      仅主机使用的动力学、传感器和场景模型
tests/           单元、集成和随机回归测试
docs/            架构和设计说明
```

CMake 将 `nav_core`、`nav_sim` 和 `nav_stm32_port` 分开构建。固件只需要 `nav_core` 与 `nav_stm32_port`，不应链接 `simulation/`。

## STM32 对接路径

1. 先用 `nav_runtime_config_default` 建立字段完整的静态 `NavAppConfig`，再用实机标定值覆盖；不要原样照搬仿真参数。
2. 实现 `platform/stm32/nav_platform.h` 中的时钟、IMU、ToF、光流、双相机、局部障碍、任务命令、停靠状态、多机通信和非阻塞 FCU 指令队列。
3. 以 100–200 Hz 调用 `nav_app_step`。IMU 采样、姿态/角速度环、混控和电机输出必须运行在更高优先级。
4. 在解锁前完成坐标系、单位、时间戳回绕、传感器失效、围栏、紧急指令和停靠触点的台架测试。

低层姿态稳定不能依赖导航任务。板级 `nav_fcu_setpoint_write` 接收带有效期的速度/加速度候选目标；适配器只能选择飞控明确支持的控制量，不能直接驱动电机。双 MCU 数据所有权、时序和失联策略见[双 MCU 飞控 UART 桥接](docs/fcu_uart_bridge.md)。

## 已知边界

- 未实现真实图像中的特征提取、目标分类或 marker 解码；当前接口从像素观测开始。
- 无稠密地图、一般三维走廊搜索或完整优化规划器；局部绕行只适合少量已知障碍。
- 多机模式已实现状态帧协议、接收时效、未来冲突和确定性让行，但没有时钟同步、未来轨迹上链路、编队/任务分配和实机通信测试。
- 没有真实电机、电池、气动、接触结构或无线充电模型。
- 尚未在目标 STM32、传感器和机体上测量 RAM、最坏执行时间与控制稳定裕量。
- 30 秒预算是仿真安全约束；实机必须根据电池和比赛规则留出更大的返航裕量。
- FCU 桥接已有协议无关契约和参考字节流，但尚未绑定或验证任何具体飞控固件。

进一步说明见 [架构文档](docs/architecture.md)、[双 MCU 飞控 UART 桥接](docs/fcu_uart_bridge.md)、[PX4 蓝牙通信验证手册](docs/px4_bluetooth_validation.md)、[集群链路](docs/swarm_link.md)、[系统工作原理与项目讲解指南](docs/system_overview_and_presentation_guide.md)、[日志回放与 watchdog](docs/replay_and_watchdog.md)、[总体方案](General_Plan.md) 和 [STM32 移植说明](platform/stm32/README.md)。
