# RM_Drone_Nav

面向 RoboMaster 2027 空中机器人的任务级自主导航项目。当前仓库提供可在主机回归的 C99 算法核心、仿真与测试入口，以及 STM32 候选适配骨架；它不是已经完成实机飞行验证的固件，也不绑定最终上层计算平台、定位模块或飞控协议。

> 当前开发顺序：先文档与接口契约，再做最小适配实现，最后进入硬件和飞行验证。主机编译、单元测试或仿真成功均不等价于实机可用。

## 当前任务边界

本项目当前负责：

- 接收外部视觉模块给出的目标观测；
- 统一坐标、时间、单位、有效性和质量语义；
- 接入唯一主定位状态，完成任务管理、路径规划、末端制导和安全中止；
- 向独立飞控输出三维速度及航向类控制目标；
- 在规则、飞行能力和定位可靠性允许时提高末端接近速度；
- 保留并继续验证已有撞击检测、恢复和仿真能力。

当前不负责装甲板识别、神经网络训练、飞镖或弹丸拦截、侦察信息采集与回传、复杂撞击动力学研究，以及低层姿态、电机和混控算法。现有像素观测、相机重建、撞击恢复和停靠代码是已实现的软件资产，不因任务边界调整而删除；其中与外部视觉职责重叠的部分只能作为兼容或仿真路径，不能继续被表述为本项目必须自行完成的感知系统。

规则依据为 2026-10-15 发布的《RoboMaster 2027 机甲大师超级对抗赛比赛规则手册 V1.0.0》。正式规则规定空中机器人全自动运行并配备急停遥控器，空中机器人冲击属于攻击方式。规则中的冲击检测接触时间与面累积冲量仍为待定，因此末端接近速度和接触参数尚不能冻结。规则中关于飞出战场、飞行不稳定后的降落停桨由安全员人工处置，不作为本导航软件的规则合规功能；软件只负责自身的数据失效、指令超时、任务中止和外部中止命令响应。2026-09-09 的规则变更前瞻只作历史参考，不覆盖正式规则。

## 状态说明

| 状态 | 含义 |
|---|---|
| `Implemented` | 源码中已存在，但仍需结合后面的验证级别解释 |
| `Sim-verified` | 已在主机测试或软件在环场景中验证，不代表硬件或飞行验证 |
| `Draft` | 推荐设计，尚未成为跨组或硬件接口契约 |
| `待确认` | 依赖视觉组、硬件组、飞控选型或后续规则 |
| `Frozen` | 已评审并作为当前阶段基线；变更需同步文档和回归证据 |

当前结论：

- C99 固定内存算法核心与 `NavRuntime` 调用链：`Implemented / Sim-verified`；
- 主机闭环、故障注入、回放、遥测和 82 项 CTest 清单：`Implemented / Sim-verified`；
- STM32 `NavApp` 和板级函数：`Implemented` 的候选骨架，未做目标板与实机验证；
- ARM Linux（例如 RK3566）上层平台：`待确认`，仓库尚无对应适配层；
- 外部 VIO（例如 MicoAir MicoVIO）：`待确认`，不是导航核心必需依赖；
- PX4/MAVLink：重点候选和验证参考，尚无真实协议驱动；
- 外部视觉输出维度、坐标系、时间戳、速率和速度字段：`待确认`。

## 已实现的软件资产

| 子系统 | 当前实现与证据边界 |
|---|---|
| 共享运行时 | `core/runtime/nav_runtime.*` 统一输入健康检查、状态估计、任务、安全、规划、制导与控制调用顺序 |
| 状态估计 | IMU 预测、光流/外部里程计修正、ToF 高度融合、视觉绝对修正门限和显式健康状态 |
| 目标跟踪 | 固定内存 `TargetObs` / `TargetTrack`、置信度、观测年龄和短时预测；正式外部视觉契约尚未冻结 |
| 任务与制导 | 19 个任务状态、目标丢失处理、末端制导、返航、停靠和紧急降落 |
| 撞击处理 | 撞击检测、视觉保持、姿态恢复、速度阻尼、估计器检查和脱离；属于保留资产，不代表已经完成真实撞击验证 |
| 轨迹与安全 | 固定容量轨迹、限速限加速度、围栏、输入时效、watchdog、控制饱和和碰撞风险处理 |
| FCU 契约 | `FcuStateSnapshot`、`FcuSetpoint`、参考字节流、序号/CRC/超时与断线回归；尚未接入具体飞控 |
| 多机扩展 | 可关闭的邻机状态、未来冲突和失联保护；不属于当前最优先主线 |
| 诊断与复现 | 固定内存事件环、二进制遥测、CSV 回放和确定性故障注入 |

## 已确认的平台无关边界

`nav_core` 使用 C99、固定容量数组和有界循环，不直接包含 STM32 HAL、POSIX、线程、文件系统、ROS 2、Docker 或特定飞控 SDK。`simulation/` 和 `platform/host/` 只服务主机验证；`platform/stm32/` 是候选板级装配层。未来若采用 ARM Linux，应新增独立适配层并继续调用相同 C 接口，不应把 Linux、ROS 2 或某款 SDK 渗入算法核心。

现有三个重要接口并不等于已经冻结：

- 飞行状态：`FcuStateSnapshot` 已覆盖姿态、角速度、加速度、本地位置/速度、高度、有效位和能力；`NavState` 是核心内部状态。两者之间的正式数据所有权和唯一主定位选择仍有接入缺口。
- 目标观测：`TargetObs` 已接近三维相对目标接口，但 `NavRuntimeInput` 目前只正式接收 `PixelObs` 并在核心内重建三维位置；外部视觉已提供三维结果时会缺少无重复重建的入口。
- 飞行指令：`FcuSetpoint` 已包含位置/速度/加速度、航向、有效期和运动使能，可作为未来 `FlightCommand` 的主要复用对象；真实飞控能力与控制授权仍需适配层确认。

详细证据、数据所有权和最小改造清单见[软件架构与平台无关化评审](docs/architecture.md)。

## 构建与验证

依赖 CMake 3.16+ 和 C99 编译器。Windows MinGW 示例：

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

统一质量门禁：

```powershell
cmake -DQUALITY_BUILD_DIR=build -DQUALITY_CONFIG=Debug -P cmake/quality_gate.cmake
```

本机与 Gitee 流水线说明见[持续集成质量门禁](docs/quality_gate.md)。`build*/` 是本地生成目录，不应提交。

常用软件验证入口：

```powershell
.\build\mission_sim.exe
.\build\mission_sim.exe --list-scenarios
.\build\mission_sim.exe --scenario target-loss
.\build\mission_sim.exe --scenario local-obstacle
.\build\mission_sim.exe --scenario flow-dropout
.\build\mission_sim.exe --scenario imu-stale
.\build\mission_sim.exe --scenario watchdog-overrun
.\build\swarm_sim.exe --scenario guarded-outage
.\build\nav_replay.exe --verify tests\data\replay_stationary.csv
.\build\fcu_bridge_sim.exe
```

当前 CTest 清单为 82 项，覆盖模块、运行时输入健康、故障注入、回放与遥测、FCU 参考桥、STM32 装配桩、多机网络和随机场景。这个数字只描述当前测试清单；测试通过只能支持 `Sim-verified`，不能支持“已在硬件验证”或“正式可部署”。

## 目录边界

```text
core/            平台无关的运行时、数学、估计、任务、规划、制导和安全
perception/      相机模型、光流前端和目标/基座跟踪；不是完整识别算法
flight/          导航状态与飞控指令的协议无关类型和转换
swarm/           默认可关闭的多机扩展
platform/host/   主机仿真、回放和诊断入口
platform/stm32/  STM32 候选适配与 NavApp 装配骨架
simulation/      仅主机使用的动力学、传感器、场景和故障模型
tests/           单元、集成和确定性回归
docs/            权威设计、流程和专题说明
```

CMake 将 `nav_core`、`nav_sim` 和 `nav_stm32_port` 分开构建。任何真实平台都应只复用 `nav_core` 和对应适配层，不应链接 `simulation/`。

## 候选平台路线

- STM32：保留 `platform/stm32/`，用于评估固定内存、确定性周期和板级驱动；它不是已选定的唯一上层平台。
- ARM Linux：若最终选用 RK3566 或其他开发板，应新增 `platform/linux/` 或独立进程适配，负责设备、通信、时间和进程生命周期；算法核心继续以 C99 库形式复用。
- 飞控：PX4/MAVLink 可优先验证，但 `RMFC` 仅是内部参考字节流，不是 MAVLink，也不证明任何真实飞控已兼容。
- 定位：飞控融合、外部 VIO 或其他来源都必须先经唯一状态提供者选择和归一化；不得让两套定位结果同时成为主导航状态。

## 文档索引

- [总体计划与分阶段路线](General_Plan.md)
- [开发流程与工程规范](docs/development_workflow.md)
- [软件架构与平台无关化评审](docs/architecture.md)
- [双端飞控链路参考设计](docs/fcu_uart_bridge.md)
- [PX4 蓝牙候选验证手册](docs/px4_bluetooth_validation.md)
- [系统工作原理与讲解指南](docs/system_overview_and_presentation_guide.md)
- [持续集成质量门禁](docs/quality_gate.md)
- [故障注入与安全回归](docs/fault_injection.md)
- [日志回放与 watchdog](docs/replay_and_watchdog.md)
- [结构化遥测](docs/telemetry.md)
- [STM32 候选适配说明](platform/stm32/README.md)

历史的集群、撞击、返航和停靠研究仍保留在源码与专题文档中；它们是可复用资产和仿真证据，不自动等于当前比赛主线优先级或可部署能力。
