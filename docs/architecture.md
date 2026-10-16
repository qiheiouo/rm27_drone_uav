# 软件架构与平台无关化设计评审

评审日期：2026-10-15

评审范围：当前 `main` 的 C99 源码、CMake 目标、主机仿真、STM32 候选适配、测试与相关技术文档。FlightState 与 FlightCommand 第一阶段已经实现并通过主机验证；TargetObservation 和真实通信仍保持设计或待确认状态。

## 1. 结论摘要

现有 `nav_core` 已经具备较好的平台无关基础：C99、固定内存、有界循环、核心无 HAL/POSIX/线程/文件系统/ROS 2/具体飞控 SDK 依赖，主机与 STM32 骨架都调用 `NavRuntime`。不需要整体迁移 C++。

真正阻碍平台迁移的不是语言，而是接口尚未完全闭合：

1. `NavRuntimeInput` 的目标入口只有 `PixelObs`，核心在 `nav_runtime_step()` 内按目标尺寸重建三维位置；外部视觉若已经提供三维位置，没有正式直通入口。
2. `FlightState` 现已作为最小外部融合状态契约进入 `NavApp` 与 `NavRuntime`；静态配置保证它与原有内部估计链互斥，外部状态不会再次进入惯性积分器。
3. 当前只支持启动时选择来源，不支持自动切换；真实状态适配器、接收时间/时钟同步和可比较质量仍等待硬件与定位方案确认。
4. `FcuSetpoint` 已扩展为统一 `FlightCommand`：最终命令显式选择 velocity/acceleration 与 yaw/yaw-rate，固定内部 local XY、Z-up 坐标，并由 `NavApp` 的单一授权门决定是否产生运动包。
5. 仓库只有 `platform/host/` 与 `platform/stm32/`；ARM Linux 适配层尚不存在，不能把“可以新增”写成“已经支持”。

下一阶段只应继续 TargetObservation 契约与直通入口；真实飞控能力、坐标转换、实际接受/执行反馈和 Failsafe 仍等待硬件选型，不应在核心中猜测。

## 2. 当前构建与依赖边界

`CMakeLists.txt` 明确设置 C99，并把目标分为：

- `nav_core`：核心、感知数学、飞控中立契约与多机算法；
- `nav_sim`：动力学、传感器、场景和故障注入，只供主机使用；
- `nav_stm32_port`：`platform/stm32/nav_tasks.c` 装配骨架；
- `mission_sim`、`swarm_sim`、`nav_replay`、`nav_telemetry_dump`、`fcu_bridge_sim`：主机程序；
- 测试目标：当前 CTest 清单 84 项。

主机专用的 `stdio`、文件读写和命令行只出现在 `platform/host/` 与测试。`simulation/` 不应进入真实平台。`platform/stm32/nav_platform.h` 只声明板级函数，没有包含 STM32 HAL；因此它能在主机用桩编译，但其命名和函数集合仍是 STM32 候选装配，不是通用 Linux 接口。

## 3. 当前实际数据流

### 3.1 主机仿真

```text
simulation/*
  → platform/host/main.c 构造 NavRuntimeInput
  → nav_runtime_step()
  → NavRuntimeOutput.guidance / control
  → simulation/sim_dynamics.c
```

这条链路验证软件状态机和数值行为，不验证真实传感器、通信、飞控或机体。

### 3.2 STM32 候选装配

`platform/stm32/nav_tasks.c::nav_app_step()` 当前执行：

```text
nav_time_ms + nav_imu_read + (nav_flow_read 或 nav_flight_state_read)
  + nav_camera_target_read + nav_camera_home_read + 其他输入
  → NavRuntimeInput
  → nav_runtime_step()
  → CtrlOutput.velocity_cmd（导航期望速度）
  → FcuMotionAuthorization（唯一最终授权门）
  → fcu_setpoint_from_nav(..., explicit mode/frame, ...)
  → nav_fcu_setpoint_write()
```

`NavMissionCommand.motion_authorization_request` 是任务/平台侧授权请求，`motion_authorization_sequence` 是明确授权/撤销决策的 16 位非零序号。`NavApp` 只有在本周期运行时有效、任务处于运动状态且收到新的授权决策序号时，才生成 `motion_enabled=1` 的最终包。定位、必需输入、命令数值或发送前置条件失效都会撤销授权并生成禁用包；恢复后旧序号无论请求保持高，还是由任务代码自动低→高，都不能恢复运动。`nav_fcu_set_motion_authorized()` 只报告主机最终授权状态，不表示飞控真实 armed 或已经执行。

这是一个固定内存、非阻塞意图明确的骨架，但存在两个关键事实：

- 它按 `flight_state_source` 只读取内部 Flow 路径或外部 `FlightState` 路径；真实 `FcuBridge`/MAVLink 接收任务仍未实现；
- 禁用包、零速度和停止发送在具体飞控上的效果尚未验证；当前实现只证明主机撤销了运动授权。

### 3.3 核心输入处理

`core/runtime/nav_runtime.h::NavRuntimeInput` 当前包含：

- 必需 `ImuSample`；
- 按静态模式互斥使用的外部 `FlightState`，或原有 `FlowFrame` / `OdomSample`；
- ToF 高度；
- `PixelObs` 目标和基座像素观测；
- 障碍、邻机、任务/返航/紧急命令和停靠状态。

`core/runtime/nav_runtime.c::nav_runtime_step()` 会：

1. 检查输入年龄、重复、倒序和非有限值；
2. 内部模式在光流存在时运行 `vf_update()`，否则使用 `OdomSample`，再调用 `estimator_update()`；
3. 外部模式验证 `FlightState` 后直接形成 `NavState`，跳过 `estimator_update()`；原始 IMU 仅继续服务碰撞检测等独立安全逻辑；
4. 将 `PixelObs` 通过 `camera_reconstruct_nav()` 转成 `TargetObs`；
5. 调用 `target_tracker_update()`、任务、安全、规划、制导和位置控制；
6. 输出 `GuidanceOutput` 与 `CtrlOutput`；`CtrlOutput.velocity_cmd` 是位置/速度制导经现有位置环得到的完整导航期望速度。

外部融合飞行状态已能直接替代核心估计器输出；外部三维目标状态仍未直接进入主链。

## 4. 当前类型复用评估

### 4.1 FlightState

相关现有类型：

- `flight/flight_state/flight_state.h::FlightState` 是跨平台的最小融合状态输入，包含位置、速度、body-to-nav WXYZ 四元数、采样时间、字段有效位、坐标系和整体定位有效标志。
- `core/state_estimator/state_estimator.h::NavState` 已包含位置、速度、姿态、角速度、线加速度、yaw、yaw rate、时间戳、有效性、质量、估计模式和健康状态；适合作为核心内部权威状态。
- `flight/fcu_bridge/fcu_bridge.h::FcuStateSnapshot` 包含有效位、能力、姿态、角速度、加速度、本地位置/速度、高度、电量和飞控状态；适合作为外部飞控状态包的基础。
- `OdomSample` 只包含位姿、速度、有效标志和时间戳，适合估计器观测，不足以单独表达完整 FlightState 和来源质量。

结论：`FlightState` 只保留导航实际需要的融合位姿和有效性，不复制电量、armed、协方差或飞控能力。`FcuStateSnapshot` 可由 `fcu_state_to_flight_state()` 归一化；`NavState` 仍是核心内部状态。原始测量、外部融合状态和核心状态三层语义不得混用。

### 4.2 TargetObservation

相关现有类型：

- `perception/camera/camera.h::PixelObs`：二维像素、尺寸、可见性和时间戳；
- `perception/target_tracker/target_tracker.h::TargetObs`：三维相对位置、bearing、置信度和时间戳；
- `TargetTrack`：在跟踪输出中增加相对速度、范围、年龄和状态。

`TargetObs` 已是最接近目标接口的类型，应扩展或在其外增加薄适配元数据，不应另建完全重复结构。当前缺失：坐标系 ID、有效字段 mask、来源 ID、可选目标速度以及明确的采样/接收时间语义。

结论：`TargetObservation` 作为跨组契约建议以演进后的 `TargetObs` 为核心。`PixelObs` 保留为仿真/兼容输入；若视觉组给三维结果，适配器直接产生规范化 `TargetObs`，禁止再次调用 `camera_reconstruct_nav()`。

### 4.3 FlightCommand

相关现有类型：

- `GuidanceOutput`：位置、速度、加速度、yaw 和选择标志；
- `CtrlOutput`：当前内部位置控制器产生的完整速度命令、加速度与 yaw rate；
- `FcuSetpoint`：序号、生成时间、有效字段、速度/加速度、yaw/yaw rate、有效期、显式控制模式、显式内部坐标系和 `motion_enabled`。

结论：`FcuSetpoint` 是唯一 FlightCommand 结构，不再复制并行命令类型。`NavApp` 是正式装配路径的最终主机授权所有者；`FcuBridge` 复用同一带决策序号的授权原语。真实飞控能力选择、实际控制模式反馈、接受/执行状态和过期动作仍等待适配器与硬件验证。

## 5. 三类接口契约

FlightState 与 FlightCommand 为第一阶段已实现基线；TargetObservation 仍是后续设计建议。

| 项目 | 当前状态 |
|---|---|
| FlightState 契约、来源互斥、失效与恢复 | `已实现并验证（主机）` |
| `nav_flight_state_read()` 板级入口 | `已实现并由主机桩验证` |
| 真实 FCU/VIO 状态适配器 | `等待硬件与定位选型` |
| 自动定位源切换 | `设计预留，未实现` |
| TargetObservation 直通 | `设计预留，未实现` |
| FlightCommand 契约与主机授权门 | `Frozen / Sim-verified` |
| 真实飞控接受/执行反馈与 Failsafe | `等待飞控选型与硬件验证` |

### 5.1 FlightState 语义

状态：`Frozen / Sim-verified（仅软件契约）`。

| 字段 | 已实现语义 |
|---|---|
| `position_m` | 局部导航系位置，m |
| `velocity_mps` | 同一局部导航系线速度，m/s |
| `attitude_body_to_nav` | 机体系到导航系，WXYZ 四元数；有限且范数在允许区间 |
| `timestamp_ms` | 映射到导航本地单调毫秒时钟的采样时间 |
| `valid_mask` | 位置、速度、姿态三项均为必需字段 |
| `frame` | 当前只接受右手局部 X/Y、正 Z 向上；原点与航向在一次运行中保持稳定 |
| `valid` | 定位模块对整份融合状态的有效判定；为 0 时不得沿用旧状态继续运动 |

`NavRuntimeConfig.flight_state_source` 只允许 `INTERNAL_ESTIMATOR` 或 `EXTERNAL`。内部模式保留原 IMU + Flow/Odom + ToF 估计链；外部模式要求 `FlightState`，拒绝同时提供 Flow/Odom，并直接写入核心 `NavState`。原始 IMU 仍为必需输入，但只用于碰撞检测等独立逻辑，不参与外部状态的二次融合。

本轮未加入协方差、质量分数和逐包 `source_id`：现有导航计算不消费协方差；不同定位器的质量不可直接比较；活动来源由启动配置唯一确定。链路接收时间属于未来真实适配器/FcuBridge，不塞入融合状态样本。自动来源切换未设计，也未实现。

### 5.2 TargetObservation 语义

最小字段：

| 字段 | 要求 | 当前复用 |
|---|---|---|
| 可见/有效 | 与“消息收到”分离 | `TargetObs.visible` |
| 观测类型 | pixel、bearing、3D position、pose 等 | 新增枚举/valid mask |
| 坐标系 ID | camera/body/nav-local；必须显式 | 新增元数据 |
| 采样时间 | 视觉曝光/推理对应时间，不使用到达时间替代 | `timestamp_ms` 可复用 |
| 三维相对位置/bearing | 视觉组给三维时直接使用 | `rel_pos` / `bearing` |
| 可选相对速度 | 只有视觉组确实提供或适配器有定义时有效 | 需扩展；不要伪造 |
| 置信度 | 范围、无效值和门限所有者需确认 | `confidence` |
| 来源/序列 | 支持诊断重复、乱序和多相机 | 新增元数据 |

所有权：视觉组拥有识别结果；目标适配器拥有外参、坐标和时间归一化；`TargetTracker` 拥有短时跟踪与预测；制导不得回到原始图像重复识别。

输入优先级建议：规范化三维 `TargetObs` 存在时使用它；否则才走旧 `PixelObs -> camera_reconstruct_nav()`。同一帧同时存在两者时必须拒绝或按明确配置选择，不能融合两次。

### 5.3 FlightCommand 语义

状态：`Frozen / Sim-verified（主机命令生成与参考字节流）`。

| 字段/规则 | 已实现语义 |
|---|---|
| `velocity_sp` | `CtrlOutput.velocity_cmd` 经授权后的三维目标速度，m/s |
| 主控制模式 | 最终运动包只能是 `VELOCITY` 或 `ACCELERATION`；`AUTO` 只能用于桥接能力选择，不能出现在已授权运动包中 |
| 航向模式 | 最终运动包只能选择 `ANGLE` 或 `RATE`，对应有效位互斥；禁用包为 `NONE` |
| `control_frame` | 当前只接受 `LOCAL_XY_Z_UP`；PX4 NED/FRD 转换只能在未来飞控适配层完成 |
| `source_timestamp_ms` | 主机生成时刻；接收端新鲜度使用本地接收时刻加 `validity_ms` 判断，避免假定两端时钟已同步 |
| `motion_enabled` | 已通过主机最终授权门；不表示飞控 armed、接受或执行 |
| 禁用包 | `motion_enabled=0`、`valid_mask=0`、1 ms 协议占位有效期；接收端先应用禁用，再判断普通运动命令 TTL；其真实飞控效果待验证 |
| 数值检查 | 有效字段必须有限；NaN/Inf、模式/有效位不匹配、未知 frame 均拒绝成为运动包 |

正式链路只有一处最终主机授权：`NavApp::motion_authorization`。授权成立需同时满足本周期 FlightState/必需输入有效、`NavRuntimeOutput.step_valid`、任务处于运动状态、命令模式/数值合法、请求为高以及新的非零 `motion_authorization_sequence`。任何前置条件失败都会撤销授权；恢复定位只恢复计算能力，不恢复授权。同一决策序号下自动拉低再拉高仍保持禁用；只有任务/平台侧作出新的明确授权决策并递增序号才可恢复。同周期故障会读取并消费新的决策序号，以故障拒绝该次决策。

定位输入被拒绝时，任务 FSM 保留原状态，不会在该失败周期进入 `MS_ESTIMATOR_LOST`；旧实现的制导/目标内部状态也可能被保留。当前实现会清零导航期望速度/控制、重置目标跟踪和末端制导状态，并由 `NavApp` 撤销最终授权。下一有效定位到来后，FSM 仍可继续原任务并重新计算期望速度，但没有新的授权决策序号只能发送禁用包。因此“定位恢复”与“恢复运动”已分离。

任务侧把授权请求拉低可立即撤销后续运动包；重新拉高时还必须提供新的决策序号，不能由任务状态派生的布尔跳变自动恢复。授权门不会擅自把任务 FSM 重置到停机、悬停或降落状态，因此明确重授权后原有非目标航点任务仍可继续。安全员人工停桨由安全员和独立飞控链路负责；导航仍负责状态有效性、边界/输出约束和故障后的停止授权。具体飞控收到禁用包、零速度、超时或停止发送后会悬停、降落还是停机，在实测前均为待确认。

## 6. 坐标、时间和单位建议

状态：FlightState 与 FlightCommand 内部契约为 `Frozen`；外部视觉与具体飞控坐标映射仍为 `Draft / 待确认`。

- 内部导航局部系建议使用右手 z-up，SI 单位；初始化 +X 的任务含义需由场地/任务配置给出。
- 机体系建议右手 FLU（+X 前、+Y 左、+Z 上）；若飞控使用 NED/FRD，只能在飞控适配器中转换。
- 四元数顺序和方向必须在字段名/测试向量中定义，例如 body-to-nav，不允许只写 `attitude`。
- yaw 和 yaw rate 使用 rad、rad/s；位置/速度/加速度使用 m、m/s、m/s²。
- 核心继续使用单调 `uint32_t` 毫秒并验证回绕；适配器保留采样时间与接收时间，必要时做时钟偏移估计。
- 外参由目标适配层单点拥有。视觉组若输出相机系三维位置，导航适配器转换一次；视觉组若已输出约定导航系，不再二次变换。

`FlightState` 已冻结 body-to-nav 四元数方向与 WXYZ 顺序；具体适配器仍必须确认外部 X/Y 朝向、body 轴和原点，再转换到该内部契约。`FcuStateSnapshot` 或 MAVLink 数据未经这一步不得直接输入运行时。

## 7. 数据与控制所有权

| 数据/动作 | 唯一所有者 | 其他模块权限 |
|---|---|---|
| 原始视觉识别 | 外部视觉组 | 导航只消费结果和质量 |
| 视觉外参/坐标归一化 | TargetObservationAdapter | 核心不解析相机/通信协议 |
| 主飞行状态选择 | FlightStateProvider | 其他来源只能作为候选/备用 |
| 任务状态、规划和末端制导 | NavRuntime | 平台不得再并行运行另一套任务制导 |
| 导航期望速度 | `NavRuntime` / `CtrlOutput.velocity_cmd` | 平台不得另算任务级速度 |
| 最终主机运动授权与 `FcuSetpoint` | `NavApp::motion_authorization` | 适配器只能转换或更严格地禁用 |
| 飞控接受/执行的运动 | 真实飞控 | 当前主机代码没有执行反馈，不得宣称已执行 |
| 姿态、角速度、电机和实际 armed | 飞控 | 导航只能请求或读取状态 |
| 人工紧急停桨 | 安全员与独立飞控链路 | 导航接收中止结果，不实现遥控逻辑 |

必须禁止：

- 两个模块同时发送最终速度 setpoint；
- 两套定位未经显式仲裁同时成为主状态；
- 外部三维目标再次按像素尺寸重建；
- 导航把“请求解锁”当作“飞控已解锁”；
- 适配器在命令过期后继续沿用最后速度。

## 8. STM32 与 ARM Linux 适配边界

### 8.1 STM32 候选适配层

应实现：

- 单调时钟与回绕安全比较；
- 传感器/飞控状态接收、固定容量缓存和唯一快照；
- 坐标、单位、时间和有效位转换；
- 非阻塞 FCU TX、外部中止输入、日志和遥测队列；
- 固定调度预算、栈/静态 RAM/最坏执行时间测量；
- DMA/中断与任务优先级，但不把 HAL 暴露给 `nav_core`。

现有 `platform/stm32/nav_platform.h` 和 `NavApp` 可保留为起点。需要最小补齐的是统一状态提供者，而不是整体改写。

### 8.2 ARM Linux 候选适配层

应实现：

- `CLOCK_MONOTONIC` 或等价单调时间映射；
- MAVLink/厂商 SDK/外部 VIO/视觉 IPC 的进程与连接生命周期；
- 有界接收队列、最新快照和超时，不让阻塞 IO 进入核心周期；
- 坐标、时间、单位和协议转换；
- 进程 watchdog、日志和配置加载；
- 与 C 核心之间的固定 C ABI。

Linux 可以在适配层使用线程和动态内存，但不得把线程调度、STL 容器、异常或 SDK 类型传入 C99 核心。

## 9. C++ 使用结论

不需要整体迁移 C++。保留 C99 核心的理由：

- 现有算法和 84 项测试都围绕该 ABI；
- 固定内存和有界行为适合 MCU，也能作为 Linux 上的稳定库；
- 当前缺口是接口和数据所有权，不是语言表达能力；
- 整体迁移会制造大范围差异，却不解决硬件、坐标、时间和协议不确定性。

可以新增 C++17/20 的场景：

- Linux 上的 MAVLink/厂商 SDK 封装；
- 外部 VIO 或视觉中间件客户端；
- 线程、IPC、RAII 和复杂资源生命周期；
- 仅主机使用的工具。

边界要求：`extern "C"`、固定宽度整数、显式长度缓冲区、错误码、调用者明确所有权；不跨接口传 STL、异常或裸资源句柄。

## 10. 接口一致性验证

保留现有主机仿真，并增加以下最小测试设计：

1. FlightState 规范向量已经过 `NavRuntime` 与 STM32 `NavApp` 主机桩；未来 Linux 适配器必须复用同一向量。
2. 覆盖 NED/FRD 到内部 z-up/FLU 的静止、倾斜、平移和 yaw 测试向量。
3. 覆盖采样时间、接收时间、32 位回绕、过期、重复、倒序和来源重启。
4. 同一目标分别以 pixel 兼容路径和已知三维路径输入，验证不会重复重建；视觉组实际三维样本走回放测试。
5. 同一周期两种主状态的拒绝已覆盖；两种目标输入互斥留待 TargetObservation 阶段。
6. 已检查 `FcuSetpoint` 的速度/yaw 互斥模式、frame、有效期、非有限值、`motion_enabled`、序号和禁用包参考编解码。
7. 已检查 FlightState 失效、同序号请求自动低→高、显式新序号重新授权、任务侧撤销/重启、同周期故障竞争、禁用包延迟处理和桥接链路重同步；真实飞控队列、进程重启和 Failsafe 仍需 HIL/台架。

这些测试先在主机运行；只有真实协议和硬件确定后才增加 SIL/HIL。不能用适配器桩测试替代实物通信证据。

## 11. 十项评审问题逐项回答

### 1. 哪些部分已经平台无关？

`core/`、大部分 `perception/`、`flight/fcu_bridge/`、`swarm/` 使用 C99 和固定内存，没有平台 HAL/OS 依赖。`NavRuntime` 是共享装配入口，CMake 也把 `nav_core` 与 `nav_sim`、`nav_stm32_port` 分开。

### 2. 哪些模块直接依赖平台或通信？

`platform/host/` 依赖主机命令行和文件 IO；`simulation/` 依赖主机模型；`platform/stm32/nav_tasks.c` 依赖 `nav_platform.h` 的板级函数集合。`RMFC` 编解码是参考协议实现但仍为平台中立；真实 UART/MAVLink 驱动尚不存在。`px4_bluetooth_validation.md` 是候选验证说明，不是已实现依赖。

### 3. 定位状态如何进入主链，缺口是什么？

默认模式仍由必需 IMU、可选 Flow/Odom 和 ToF 进入 `estimator_update()`。外部模式由 `NavApp.nav_flight_state_read()` 填充 `NavRuntimeInput.flight_state`，经时间、数值、frame、四元数和有效位检查后直接形成 `NavState`，不调用估计器积分。缺口是尚无真实设备适配、时钟同步和硬件验证。

### 4. 如何支持不同定位来源？

候选适配器先归一化为 `FlightState`，再用 `flight_state_source` 在启动时选择内部估计或外部融合状态。当前不自动切换；若以后需要备用源，必须另行设计可观测切换状态机，不能在本接口上静默覆盖。

### 5. 如何接收视觉目标而不重复三维重建？

演进 `TargetObs` 为正式规范化目标观测，并给 `NavRuntimeInput` 增加该入口；存在规范化三维观测时直接送 `target_tracker_update()`。旧 `PixelObs` 仅在视觉组提供二维结果或仿真兼容时运行 `camera_reconstruct_nav()`，两条路径互斥。

### 6. 如何设计统一速度控制输出？

复用 `FcuSetpoint` 作为 FlightCommand。`CtrlOutput.velocity_cmd` 是导航期望速度；`NavApp` 通过单一、带明确决策序号的授权门后才把它写成已授权 `velocity_sp`。最终包在 velocity/acceleration 与 yaw/yaw-rate 中分别只能选择一种，带本地 z-up frame、生成时间和有效期。过期与失联后的真实动作由飞控最终处理，当前只验证主机不继续授权。

### 7. STM32 与 ARM Linux 各需要什么？

STM32 负责固定内存缓存、DMA/中断、非阻塞驱动、周期和资源测量；Linux 负责协议/SDK、线程/IPC、进程生命周期、单调时钟和有界队列。两者共同承担坐标/时间/单位/有效位转换，并调用相同 C99 核心。

### 8. 是否需要 C++？

核心不需要。Linux 的 SDK、通信、IPC 和资源管理可以使用 C++17/20，通过稳定 C ABI 连接。不存在为了候选 RK3566 整体迁移 C++ 的理由。

### 9. 如何保留主机仿真并验证适配一致性？

继续以 `nav_core` + `nav_sim` 为基线，新增与真实平台无关的规范向量、回放和适配器桩测试。同一标准化状态、目标和命令应在 host/STM32/Linux 边界得到一致字段和失效结果。

### 10. 当前结构性迁移阻碍是什么？

依严重性排序：目标只有像素正式入口；真实 FlightState 适配器尚不存在；FlightCommand 尚未与真实飞控能力、armed/接受/执行反馈和 Failsafe 闭合；外部设备时钟同步未实现；Linux 适配不存在；自动来源切换尚未设计。FlightState 与 FlightCommand 的主机契约、静态所有权和失效测试已完成。

## 12. 改造分类

### 可以保留不动

- C99 `nav_core` 与算法实现；
- `NavState`、`TargetTrack`、`GuidanceOutput`、`CtrlOutput` 的内部用途；`CtrlOutput` 仅增加完整速度命令字段；
- `FcuStateSnapshot` / `FcuSetpoint` 的大部分字段和参考测试；
- 主机仿真、故障注入、回放、遥测和现有测试；
- 撞击恢复、停靠和多机代码作为保留资产。

### 建议最小修改

- 让规范化 `TargetObs` 正式进入 `NavRuntimeInput`，并与 `PixelObs` 互斥；
- 硬件选型后把一个 FcuBridge/VIO 适配器接到已存在的 `nav_flight_state_read()`；
- 将未来平台实现接到 `nav_fcu_set_motion_authorized`，不得把它映射成飞控真实 armed 反馈；
- 让真实命令适配器消费飞控能力与活动状态，并提供接受/执行反馈；
- 为真实适配器补充外部 frame 转换、采样/接收时间和接口一致性测试；

### 硬件确定后才能开发

- MAVLink/厂商协议和真实设备驱动；
- STM32 HAL/RTOS、ARM Linux 进程和外部 VIO SDK；
- 时间同步、带宽、最坏延迟、控制模式和飞控失联配置；
- 接近速度、接触参数和硬件资源预算。

### 需要外部组确认

- 视觉二维/三维输出、frame、时间、速率、置信度和速度；
- 飞控状态/能力/控制模式和实际 armed 反馈；
- 上层计算平台与定位来源；
- 规则待定冲击参数和机体接触能力；
- 安全员人工停桨链路向导航提供何种中止状态与确认反馈；这不替代导航自身的有效性和边界检查。

## 13. 验证边界

当前 84 项 CTest 和主机场景可以证明代码在已建模输入下的确定性、FlightState 直通/失效处理、FlightCommand 最终主机授权和组合逻辑。它们不能证明：

- STM32 或 ARM Linux 的资源和实时性；
- 任一真实 VIO、视觉模块或飞控兼容；
- MAVLink、UART、无线链路或时钟同步已经可用；
- 真实机体接近/撞击稳定；
- 软件已经满足比赛部署条件。

安全员人工执行停桨、飞出战场和飞行不稳定后的最终处置，独立于导航软件。导航软件仍验收状态/目标/命令契约、围栏和输出约束、失效后的停止授权及外部中止响应；当前主机证据不证明真实飞控具备悬停、降落或停桨 Failsafe。
