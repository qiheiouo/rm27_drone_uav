# RM_Drone_Nav

面向 RoboMaster 2027 空中机器人的任务级自主导航项目。当前仓库提供可在主机回归的 C99 算法核心、仿真与测试入口、STM32 候选适配骨架，以及可选的 PX4/MAVLink PC 适配；它不是已经完成 PX4 SITL 或实机飞行验证的固件，也不绑定最终上层计算平台或定位模块。

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

规则依据为 2026-10-15 发布的《RoboMaster 2027 机甲大师超级对抗赛比赛规则手册 V1.0.0》。正式规则规定空中机器人全自动运行并配备急停遥控器，空中机器人冲击属于攻击方式。规则中的冲击检测接触时间与面累积冲量仍为待定，因此末端接近速度和接触参数尚不能冻结。规则中关于飞出战场、飞行不稳定后的降落停桨由安全员人工处置，人工遥控与停桨链路不由本项目实现；但导航软件仍负责状态有效性、围栏/边界约束、指令时效、故障响应和外部中止命令响应。当前没有真实飞控 Failsafe 验证，不能据主机测试声称具备实机降落或停桨能力。2026-09-09 的规则变更前瞻只作历史参考，不覆盖正式规则。

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
- 主机闭环、故障注入、回放、遥测和 84 项 CTest 清单：`Implemented / Sim-verified`；
- `FlightState` 最小契约、静态单一来源选择、外部状态直通与失效恢复：`Implemented / Sim-verified`；
- `FlightCommand` 最小契约、单一最终主机授权、失效撤销与显式重授权：`Implemented / Sim-verified`；
- PX4/MAVLink C99 状态/速度适配和 loopback-only PC 入口：`Implemented / Sim-verified（离线协议与组合路径）`；
- PX4 SITL 起飞、悬停、航点、失联和重新授权闭环：`Modify / 未验证`（本机未安装可用 PX4 SITL 环境）；
- STM32 `NavApp` 和板级函数：`Implemented` 的候选骨架，未做目标板与实机验证；
- ARM Linux（例如 RK3566）上层平台：`待确认`，仓库尚无对应适配层；
- 外部 VIO（例如 MicoAir MicoVIO）：`待确认`，不是导航核心必需依赖；
- PX4 飞控：已选；MicoAir743v2-AIO-35A 型号已核对，实际固件版本、端口、参数和实机行为仍待读取与验证；
- 外部视觉输出维度、坐标系、时间戳、速率和速度字段：`待确认`。

## 已实现的软件资产

| 子系统 | 当前实现与证据边界 |
|---|---|
| 共享运行时 | `core/runtime/nav_runtime.*` 统一输入健康检查、静态主状态来源选择、状态估计、任务、安全、规划、制导与控制调用顺序 |
| 状态估计 | IMU 预测、光流/外部里程计修正、ToF 高度融合、视觉绝对修正门限和显式健康状态 |
| 目标跟踪 | 固定内存 `TargetObs` / `TargetTrack`、置信度、观测年龄和短时预测；正式外部视觉契约尚未冻结 |
| 任务与制导 | 19 个任务状态、目标丢失处理、末端制导、返航、停靠和紧急降落 |
| 撞击处理 | 撞击检测、视觉保持、姿态恢复、速度阻尼、估计器检查和脱离；属于保留资产，不代表已经完成真实撞击验证 |
| 轨迹与安全 | 固定容量轨迹、限速限加速度、围栏、输入时效、watchdog、控制饱和和碰撞风险处理 |
| FCU 契约 | `FcuStateSnapshot`、唯一 `FcuSetpoint`/FlightCommand、显式控制与航向模式、local z-up、带决策序号的授权；可选 PX4/MAVLink 映射已通过主机离线测试，尚未完成真实 SITL/HIL |
| 多机扩展 | 可关闭的邻机状态、未来冲突和失联保护；不属于当前最优先主线 |
| 诊断与复现 | 固定内存事件环、二进制遥测、CSV 回放和确定性故障注入 |

## 已确认的平台无关边界

`nav_core` 使用 C99、固定容量数组和有界循环，不直接包含 STM32 HAL、POSIX、线程、文件系统、ROS 2、Docker 或特定飞控 SDK。`simulation/` 和 `platform/host/` 只服务主机验证；`platform/stm32/` 是候选板级装配层。未来若采用 ARM Linux，应新增独立适配层并继续调用相同 C 接口，不应把 Linux、ROS 2 或某款 SDK 渗入算法核心。

三个重要接口处于不同成熟度：

- 飞行状态：第一阶段 `FlightState` 契约已冻结为局部右手 z-up、SI 单位、body-to-nav WXYZ 四元数和导航本地单调毫秒时间；`NavRuntimeConfig.flight_state_source` 在启动时互斥选择原有内部估计链或外部融合状态。PX4 适配器已把 NED/FRD 状态归一化后接入外部路径，真实 SITL、时钟同步和硬件验证仍未完成。
- 目标观测：`TargetObs` 已接近三维相对目标接口，但 `NavRuntimeInput` 目前只正式接收 `PixelObs` 并在核心内重建三维位置；外部视觉已提供三维结果时会缺少无重复重建的入口。
- 飞行指令：`FcuSetpoint` 已是统一 `FlightCommand`。`CtrlOutput.velocity_cmd` 是导航期望速度，`NavApp` 是正式链路的最终主机授权所有者；velocity/acceleration 与 yaw/yaw-rate 分别互斥，内部 frame 固定为 local XY、Z-up，定位恢复后必须收到新的非零授权决策序号才能重新发送运动包。PX4 适配器只在授权后生成 `SET_POSITION_TARGET_LOCAL_NED`，并分别读取 heartbeat/ACK；主机发送、PX4 接受模式、armed 与实际执行仍是四种不同状态，真实 Failsafe 待 SITL/HIL 验证。

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

当前 CTest 清单为 84 项，覆盖模块、运行时输入健康、FlightState 外部直通与恢复、FlightCommand 授权/过期/非法值/故障竞争、故障注入、回放与遥测、FCU 参考桥、STM32 装配桩、多机网络和随机场景。这个数字只描述当前测试清单；测试通过只能支持 `Sim-verified`，不能支持“已在硬件验证”或“正式可部署”。

PX4/MAVLink 为可选构建，不改变上述默认 84 项。启用后额外增加适配器、NavApp 组合路径、运行器帮助和不安全启动拒绝 4 项测试。依赖、构建命令、固定 SITL 坐标映射和安全边界见 [PX4 SITL 最小闭环适配与验证手册](docs/px4_sitl.md)。

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
- 飞控：PX4/MAVLink 已有可选的 C99 协议适配和 PC loopback SITL 入口；`RMFC` 仍只是内部参考字节流。离线消息测试不证明 PX4 SITL 或实体飞控已兼容。
- 定位：飞控融合、外部 VIO 或其他来源都必须先经唯一状态提供者选择和归一化；不得让两套定位结果同时成为主导航状态。

## 文档索引

- [总体计划与分阶段路线](General_Plan.md)
- [开发流程与工程规范](docs/development_workflow.md)
- [软件架构与平台无关化评审](docs/architecture.md)
- [双端飞控链路参考设计](docs/fcu_uart_bridge.md)
- [PX4 SITL 最小闭环适配与验证手册](docs/px4_sitl.md)
- [PX4 蓝牙候选验证手册](docs/px4_bluetooth_validation.md)
- [系统工作原理与讲解指南](docs/system_overview_and_presentation_guide.md)
- [持续集成质量门禁](docs/quality_gate.md)
- [故障注入与安全回归](docs/fault_injection.md)
- [日志回放与 watchdog](docs/replay_and_watchdog.md)
- [结构化遥测](docs/telemetry.md)
- [STM32 候选适配说明](platform/stm32/README.md)

历史的集群、撞击、返航和停靠研究仍保留在源码与专题文档中；它们是可复用资产和仿真证据，不自动等于当前比赛主线优先级或可部署能力。
