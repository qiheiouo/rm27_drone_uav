# 软件架构与平台无关化设计评审

评审日期：2026-10-15

评审范围：当前 `main` 的 C99 源码、CMake 目标、主机仿真、STM32 候选适配、测试与相关技术文档。本轮只给出设计结论和最小改造清单，不修改算法实现、运行参数或通信驱动。

## 1. 结论摘要

现有 `nav_core` 已经具备较好的平台无关基础：C99、固定内存、有界循环、核心无 HAL/POSIX/线程/文件系统/ROS 2/具体飞控 SDK 依赖，主机与 STM32 骨架都调用 `NavRuntime`。不需要整体迁移 C++。

真正阻碍平台迁移的不是语言，而是接口尚未完全闭合：

1. `NavRuntimeInput` 的目标入口只有 `PixelObs`，核心在 `nav_runtime_step()` 内按目标尺寸重建三维位置；外部视觉若已经提供三维位置，没有正式直通入口。
2. `FcuStateSnapshot`、`FcuBridge` 和 `fcu_state_to_nav_samples()` 已存在，但 `platform/stm32/nav_tasks.c` 没有把它们接入 `NavApp` 主循环；文档描述的状态桥仍主要停留在契约和独立测试层。
3. 现有 `NavState` 能表达核心飞行状态，但没有“唯一主状态来源、来源切换、坐标标识、采样时间与质量”的完整外部契约。
4. `FcuSetpoint` 已很接近统一 `FlightCommand`，但 `NavApp` 直接以 `FCU_CONTROL_AUTO` 生成命令，尚未消费真实飞控能力、活动状态或控制授权反馈。
5. 仓库只有 `platform/host/` 与 `platform/stm32/`；ARM Linux 适配层尚不存在，不能把“可以新增”写成“已经支持”。

因此下一步应优先冻结三个接口的字段和所有权，再做最小适配，不应重构现有核心。

## 2. 当前构建与依赖边界

`CMakeLists.txt` 明确设置 C99，并把目标分为：

- `nav_core`：核心、感知数学、飞控中立契约与多机算法；
- `nav_sim`：动力学、传感器、场景和故障注入，只供主机使用；
- `nav_stm32_port`：`platform/stm32/nav_tasks.c` 装配骨架；
- `mission_sim`、`swarm_sim`、`nav_replay`、`nav_telemetry_dump`、`fcu_bridge_sim`：主机程序；
- 测试目标：当前 CTest 清单 82 项。

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
nav_time_ms + nav_imu_read + nav_tof_read + nav_flow_read
  + nav_camera_target_read + nav_camera_home_read + 其他输入
  → NavRuntimeInput
  → nav_runtime_step()
  → fcu_setpoint_from_nav(..., FCU_CONTROL_AUTO, ...)
  → nav_fcu_setpoint_write()
```

失败时 `nav_app_disable_fcu()` 发送禁用 setpoint，并调用 `nav_fcu_set_armed(0)`。成功时发送 setpoint 后调用 `nav_fcu_set_armed(runtime.output.armed)`。

这是一个固定内存、非阻塞意图明确的骨架，但存在两个关键事实：

- 它直接从多个板级读取函数拼装状态，没有调用 `FcuBridge`；
- `nav_fcu_set_armed()` 的名称混合了“导航请求”和“飞控真实 armed 状态”，后续协议冻结时必须澄清，不应让导航覆盖飞控最终权限。

### 3.3 核心输入处理

`core/runtime/nav_runtime.h::NavRuntimeInput` 当前包含：

- 必需 `ImuSample`；
- 可选 `FlowFrame` 或 `OdomSample`；
- ToF 高度；
- `PixelObs` 目标和基座像素观测；
- 障碍、邻机、任务/返航/紧急命令和停靠状态。

`core/runtime/nav_runtime.c::nav_runtime_step()` 会：

1. 检查输入年龄、重复、倒序和非有限值；
2. 在光流存在时运行 `vf_update()`，否则使用 `OdomSample`；
3. 调用 `estimator_update()` 形成 `NavState`；
4. 将 `PixelObs` 通过 `camera_reconstruct_nav()` 转成 `TargetObs`；
5. 调用 `target_tracker_update()`、任务、安全、规划、制导和位置控制；
6. 输出 `GuidanceOutput` 与 `CtrlOutput`。

这说明当前外部三维目标状态并未直接进入主链，外部融合飞行状态也没有直接替代核心估计器的入口。

## 4. 当前类型复用评估

### 4.1 FlightState

相关现有类型：

- `flight/flight_state/flight_state.h::FlightState` 只有模式、armed 和电压，且当前没有其他 C 源码引用；不足以承担统一飞行状态。
- `core/state_estimator/state_estimator.h::NavState` 已包含位置、速度、姿态、角速度、线加速度、yaw、yaw rate、时间戳、有效性、质量、估计模式和健康状态；适合作为核心内部权威状态。
- `flight/fcu_bridge/fcu_bridge.h::FcuStateSnapshot` 包含有效位、能力、姿态、角速度、加速度、本地位置/速度、高度、电量和飞控状态；适合作为外部飞控状态包的基础。
- `OdomSample` 只包含位姿、速度、有效标志和时间戳，适合估计器观测，不足以单独表达完整 FlightState 和来源质量。

结论：不要再新建一套与 `NavState` / `FcuStateSnapshot` 高度重复的结构。`FlightState` 应先作为接口语义名称：外部适配器基于 `FcuStateSnapshot` 或外部 VIO数据归一化，唯一状态提供者再映射为现有核心输入/状态。若后续确实需要新 ABI，应从现有字段演进并删除或弃用当前未使用的简化 `FlightState`，避免三个并行权威结构。

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
- `CtrlOutput`：当前内部位置控制器产生的加速度与 yaw rate；
- `FcuSetpoint`：序号、生成时间、有效字段、位置/速度/加速度、yaw/yaw rate、有效期、控制偏好和 `motion_enabled`。

结论：`FcuSetpoint` 已经满足大部分 `FlightCommand` 要求，应作为统一外部指令的基础，不需要再复制一个命令结构。需要冻结的是：`motion_enabled` 的授权语义、唯一命令所有者、飞控能力选择、过期动作、模式反馈和导航请求与飞控实际 armed 状态的区别。

## 5. 三类接口的 Draft 契约

以下为设计建议，不是本轮新增结构体或跨组已确认协议。

### 5.1 FlightState 语义

最小字段：

| 字段 | 要求 | 可复用来源 |
|---|---|---|
| `source_id` | 飞控融合、外部 VIO、内部估计等枚举；任一时刻仅一个 active | 新增元数据 |
| `frame_id` | 明确导航局部系或机体系，禁止靠注释猜测 | 新增元数据 |
| `sample_timestamp` | 传感/估计采样时间 | `source_timestamp_ms` / `timestamp_ms` |
| `received_timestamp` | 本机接收时间，用于链路年龄 | `FcuBridge.received_at_ms` |
| 位置/速度/姿态 | SI 单位、四元数顺序明确 | `NavState` / `FcuStateSnapshot` |
| 角速度/线加速度 | 明确机体系与比力/线加速度语义 | 现有字段 |
| `valid_mask` | 每个字段独立有效 | `FcuStateSnapshot.valid_mask` |
| `quality` / health | 可比较但不假设所有来源同一算法 | `NavState.quality/status` + 适配映射 |

所有权：平台状态适配器拥有协议解析和坐标/时间转换；`FlightStateProvider` 拥有唯一来源选择；`NavRuntime` 只消费一个已选状态或其现有采样形式。

切换规则：备用来源不能静默覆盖主来源。切换必须经过显式状态机，至少记录旧/新来源、原因、时间、位置/速度跳变和是否需要保持/中止。硬件未定前不冻结自动切换阈值。

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

以 `FcuSetpoint` 为基础：

| 字段 | 要求 |
|---|---|
| 速度 | 当前首选外部控制量，三维、SI 单位 |
| 航向 | 飞控确认后选择 yaw 或 yaw rate，不能同时含糊生效 |
| 生成时间/有效期 | 超时后飞控侧停止沿用最后命令 |
| 控制模式 | velocity/acceleration/position 只选飞控明确支持的一种主模式 |
| `motion_enabled` | 表示导航是否授权本条运动，不代表飞控已经解锁 |
| 序号/来源 | 支持重复、乱序、重启与诊断 |
| 失效动作 | 由飞控/平台适配器实现，导航只发禁用或停止授权 |

所有权：`NavRuntime` 是任务级运动意图的唯一生产者；命令适配器只做能力选择、坐标/单位/协议转换和更严格的禁用；飞控拥有最终低层控制、解锁、限幅和失联动作。规则中安全员人工停桨由安全员和飞控链路负责，不属于本导航软件实现；本软件只需在接到外部中止状态后停止输出有效运动授权。

## 6. 坐标、时间和单位建议

状态：`Draft / 待视觉组与飞控组确认`。

- 内部导航局部系建议使用右手 z-up，SI 单位；初始化 +X 的任务含义需由场地/任务配置给出。
- 机体系建议右手 FLU（+X 前、+Y 左、+Z 上）；若飞控使用 NED/FRD，只能在飞控适配器中转换。
- 四元数顺序和方向必须在字段名/测试向量中定义，例如 body-to-nav，不允许只写 `attitude`。
- yaw 和 yaw rate 使用 rad、rad/s；位置/速度/加速度使用 m、m/s、m/s²。
- 核心继续使用单调 `uint32_t` 毫秒并验证回绕；适配器保留采样时间与接收时间，必要时做时钟偏移估计。
- 外参由目标适配层单点拥有。视觉组若输出相机系三维位置，导航适配器转换一次；视觉组若已输出约定导航系，不再二次变换。

现有 `FcuStateSnapshot` 注释只冻结了“local X/Y、正 Z 向上”，尚未完整定义 X/Y 朝向、body 轴和四元数方向，因此当前不能声称坐标契约已 Frozen。

## 7. 数据与控制所有权

| 数据/动作 | 唯一所有者 | 其他模块权限 |
|---|---|---|
| 原始视觉识别 | 外部视觉组 | 导航只消费结果和质量 |
| 视觉外参/坐标归一化 | TargetObservationAdapter | 核心不解析相机/通信协议 |
| 主飞行状态选择 | FlightStateProvider | 其他来源只能作为候选/备用 |
| 任务状态、规划和末端制导 | NavRuntime | 平台不得再并行运行另一套任务制导 |
| 最终导航运动指令 | FlightCommandOwner（由 NavRuntime 输出形成） | 适配器只能转换或禁用 |
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

- 现有算法和 82 项测试都围绕该 ABI；
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

1. 同一 FlightState 规范向量分别经过 host、STM32 桩和未来 Linux 适配器，比较核心可见字段。
2. 覆盖 NED/FRD 到内部 z-up/FLU 的静止、倾斜、平移和 yaw 测试向量。
3. 覆盖采样时间、接收时间、32 位回绕、过期、重复、倒序和来源重启。
4. 同一目标分别以 pixel 兼容路径和已知三维路径输入，验证不会重复重建；视觉组实际三维样本走回放测试。
5. 检查同一周期两种主状态或两种目标输入同时出现时被明确拒绝/选择。
6. 检查 `FcuSetpoint` 的速度/yaw、有效期、`motion_enabled`、序号和禁用包在不同适配器编码后语义一致。
7. 飞控状态过期、命令队列拒绝、外部中止输入和进程/链路重启后，导航不继续授权旧运动命令。

这些测试先在主机运行；只有真实协议和硬件确定后才增加 SIL/HIL。不能用适配器桩测试替代实物通信证据。

## 11. 十项评审问题逐项回答

### 1. 哪些部分已经平台无关？

`core/`、大部分 `perception/`、`flight/fcu_bridge/`、`swarm/` 使用 C99 和固定内存，没有平台 HAL/OS 依赖。`NavRuntime` 是共享装配入口，CMake 也把 `nav_core` 与 `nav_sim`、`nav_stm32_port` 分开。

### 2. 哪些模块直接依赖平台或通信？

`platform/host/` 依赖主机命令行和文件 IO；`simulation/` 依赖主机模型；`platform/stm32/nav_tasks.c` 依赖 `nav_platform.h` 的板级函数集合。`RMFC` 编解码是参考协议实现但仍为平台中立；真实 UART/MAVLink 驱动尚不存在。`px4_bluetooth_validation.md` 是候选验证说明，不是已实现依赖。

### 3. 定位状态如何进入主链，缺口是什么？

当前 `NavRuntimeInput` 通过必需 IMU、可选 Flow/Odom 和 ToF 进入 `estimator_update()`，形成 `NavState`。`FcuStateSnapshot` 可以由 `fcu_state_to_nav_samples()` 转成 IMU/Odom/height，但该路径未接入 `NavApp`。外部已融合 FlightState 没有包含来源/坐标/质量和唯一仲裁的正式入口，这是主要缺口。

### 4. 如何支持不同定位来源？

在平台边界建立唯一 `FlightStateProvider`，候选适配器先归一化，再显式选择一个 active 来源。优先复用 `FcuStateSnapshot`、`NavState` 和 `OdomSample`；只有现有字段确实不足时才演进类型。来源切换必须可观测、可拒绝，并能触发保持/中止。

### 5. 如何接收视觉目标而不重复三维重建？

演进 `TargetObs` 为正式规范化目标观测，并给 `NavRuntimeInput` 增加该入口；存在规范化三维观测时直接送 `target_tracker_update()`。旧 `PixelObs` 仅在视觉组提供二维结果或仿真兼容时运行 `camera_reconstruct_nav()`，两条路径互斥。

### 6. 如何设计统一速度控制输出？

复用 `FcuSetpoint` 作为 FlightCommand 基础，默认冻结“速度 + yaw/yaw rate 二选一 + 生成时间 + 有效期 + motion_enabled”。适配器按真实飞控能力映射，不能同时重复闭合速度和加速度环；过期与失联由飞控最终处理。

### 7. STM32 与 ARM Linux 各需要什么？

STM32 负责固定内存缓存、DMA/中断、非阻塞驱动、周期和资源测量；Linux 负责协议/SDK、线程/IPC、进程生命周期、单调时钟和有界队列。两者共同承担坐标/时间/单位/有效位转换，并调用相同 C99 核心。

### 8. 是否需要 C++？

核心不需要。Linux 的 SDK、通信、IPC 和资源管理可以使用 C++17/20，通过稳定 C ABI 连接。不存在为了候选 RK3566 整体迁移 C++ 的理由。

### 9. 如何保留主机仿真并验证适配一致性？

继续以 `nav_core` + `nav_sim` 为基线，新增与真实平台无关的规范向量、回放和适配器桩测试。同一标准化状态、目标和命令应在 host/STM32/Linux 边界得到一致字段和失效结果。

### 10. 当前结构性迁移阻碍是什么？

依严重性排序：目标只有像素正式入口；主状态没有唯一来源仲裁；FcuBridge 未进入 NavApp 主链；命令能力/授权未与真实飞控状态闭合；坐标与四元数契约不完整；Linux 适配不存在；`FlightState` 简化类型未使用且容易形成重复概念；时间同步与来源切换尚无接口测试。

## 12. 改造分类

### 可以保留不动

- C99 `nav_core` 与算法实现；
- `NavState`、`TargetTrack`、`GuidanceOutput`、`CtrlOutput` 的内部用途；
- `FcuStateSnapshot` / `FcuSetpoint` 的大部分字段和参考测试；
- 主机仿真、故障注入、回放、遥测和现有测试；
- 撞击恢复、停靠和多机代码作为保留资产。

### 建议最小修改

- 让规范化 `TargetObs` 正式进入 `NavRuntimeInput`，并与 `PixelObs` 互斥；
- 建立唯一 FlightState 来源/选择器并把 FcuBridge 接入平台装配；
- 澄清/替换 `nav_fcu_set_armed` 的请求语义，避免冒充飞控真实状态；
- 让命令生成使用飞控能力与活动状态；
- 补充 frame、来源、有效位、采样/接收时间和接口一致性测试；
- 将未使用的简化 `FlightState` 合并、弃用或明确仅代表飞控模式，避免重名。

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
- 安全员人工停桨链路只需向导航提供何种中止状态。

## 13. 验证边界

当前 82 项 CTest 和主机场景可以证明代码在已建模输入下的确定性、失效处理和组合逻辑。它们不能证明：

- STM32 或 ARM Linux 的资源和实时性；
- 任一真实 VIO、视觉模块或飞控兼容；
- MAVLink、UART、无线链路或时钟同步已经可用；
- 真实机体接近/撞击稳定；
- 软件已经满足比赛部署条件。

规则中由安全员人工执行的停桨、飞出战场和飞行不稳定处置不纳入导航软件功能验收。导航软件只验收自身状态/目标/命令契约、失效后的停止授权，以及外部中止输入的响应。
