# 飞控状态与指令链路参考设计

> 状态：FlightCommand 软件契约为 `Frozen / Sim-verified`；`RMFC` 传输仍为 `Draft / 候选方案`。PX4/MAVLink 可选适配已在 Ubuntu 24.04、PX4 v1.17.0、Gazebo Sim 8.10.0 的 `gz_x500` 上完成真实 SITL 闭环验证；HIL 和实体飞控仍未验证。本文记录通用 `FcuBridge`、`FcuStateSnapshot`、`FcuSetpoint` 和参考字节流；PX4 专项约定见 `docs/px4_sitl.md`。

## 目标与边界

上层导航与飞控可以位于同一硬件，也可以通过 UART、CAN、以太网或其他链路连接。当前桥接层只固定两端的软件语义：

```text
飞控传感器/估计器
  → 带采样时间的 FcuStateSnapshot
  → fcu_state_to_flight_state（坐标/时间已由适配器归一化）
  → NavRuntime 外部状态直通、任务、规划与制导
  → CtrlOutput.velocity_cmd（导航期望速度）
  → FcuMotionAuthorization（最终主机授权）
  → 有有效期的 FcuSetpoint（已授权速度或禁用包）
  → 飞控速度/加速度接口
  → 姿态环、角速度环、混控和电机
```

姿态环、角速度环、混控和电机输出必须留在飞控，不经过上层链路闭环。导航侧
不得发送电机值。飞控必须保留解锁、限幅、外部控制丢失和最终急停权限。

本通用模块不绑定 PX4、ArduPilot、Betaflight 或具体 UART HAL。当前可选的
`flight/px4_mavlink/` 把统一结构映射到 MAVLink common 消息，但没有改变通用契约；`RMFC`
二进制帧只是自定义固件和主机测试可用的参考编码，不要求 PX4 改用它。

## 数据所有权

IMU 应由飞控或明确的唯一硬件所有者采集，不能让两个处理器同时驱动同一 SPI/I2C 设备。若光流和测距已经由
飞控融合，导航优先消费飞控输出的本地位置、速度和高度；原始加速度与角速度仍可用于
导航侧撞击检测。目标相机、基座相机或独立视觉模块可以连接上层平台；本项目只消费规范化结果。

`FcuStateSnapshot` 使用以下约定：

- 本地导航坐标系：x/y 为场地水平轴，z 向上；
- `attitude` 表示机体系到导航系的四元数，顺序为 w/x/y/z；
- `angular_velocity` 是机体系 rad/s；
- `linear_acceleration` 是飞控 IMU 提供的机体系比力，单位 m/s²；
- 位置、速度和高度使用 SI 单位；
- `source_timestamp_ms` 是传感器采样时间，不是 UART 接收时间；
- 电量范围为 0–1，未知时清除相应有效位。

MAVLink 常用 NED/FRD，而导航核心 z 向上。坐标变换必须在适配器完成，不能把未经转换的
NED 数据直接交给 `NavRuntime`。在两块 MCU 尚未完成时钟同步时，链路超时使用本地
`received_at_ms`；源时间戳只用于诊断。完成时间同步后，再把源时间映射到导航本地时钟并
传给 `fcu_state_to_flight_state`。该函数使用导航本地采样时间生成 `FlightState`，外部模式会直接形成 `NavState`，不会再次进入惯性积分器。`fcu_state_to_nav_samples` 仅为既有桥接实验保留，不应作为飞控已融合状态的默认接入方式。

## 状态和指令契约

状态有效位分别标记姿态、角速度、线加速度、本地位置、本地速度、高度和电量。能力位声明
飞控是否接受位置、速度、加速度、偏航角、偏航角速度和外部控制。未知或非有限字段、非法
布尔值、越界电量和畸形四元数会被拒绝。

`FcuSetpoint` 保留位置字段用于参考编码兼容，但当前已授权运动包执行严格互斥：主控制量只能
选择 velocity 或 acceleration，航向只能选择 yaw angle 或 yaw rate。`AUTO` 只允许作为
`FcuBridgeConfig` 的能力选择请求，不能出现在最终运动包中；模式、有效位不匹配以及 NaN/Inf
都会被拒绝。当前内部 `control_frame` 只允许右手 local XY、Z-up：

- 支持外部速度控制时，发送 `CtrlOutput.velocity_cmd` 和选定航向目标；该速度包含现有位置/速度制导形成的完整导航期望，不只是速度前馈；
- 只支持外部加速度控制时，发送 `CtrlOutput.accel_cmd` 和偏航角速度；
- 不支持这两种接口时，不得悄悄退化成电机输出；需要重新评估飞控选型或单独设计遥控量适配。

`NavApp` 是正式装配路径唯一的最终主机授权所有者。`NavMissionCommand.motion_authorization_request`
是请求，不是飞控 armed 反馈；`motion_authorization_sequence` 是任务/平台侧每次明确授权或撤销决策
递增的 16 位非零序号，零表示没有决策。只有本周期状态有效、任务处于运动状态、请求为高且序号
相对上次决策为新时，`FcuMotionAuthorization` 才允许生成 `motion_enabled=1` 的包。运行时输入、命令
数值/模式或发送前置条件失效会撤销授权；同一序号下持续为高或自动低→高都不能恢复运动。故障周期
仍读取并消费新决策，以故障拒绝该次决策；恢复后必须再作出新的明确授权。序号按 16 位回绕比较，
任务命令生产者重启时必须与 `NavApp` 重新同步，不能静默把旧序号当成新决策。

候选装配默认使用 velocity + yaw angle，可分别由 `NAV_APP_FCU_CONTROL_MODE` 和
`NAV_APP_FCU_YAW_CONTROL` 在板级编译配置中选择。yaw angle 与 yaw rate 的未来最终选择尚未冻结；
适配器负责按真实飞控能力选定其一，若需要二者互转，转换责任也属于适配器而不是导航核心。

每条命令带序号、生成时间和有效期。默认有效期为 100 ms，可用
`NAV_APP_FCU_COMMAND_VALIDITY_MS` 覆盖；接收侧以本地接收时刻计算年龄。禁用包为
`motion_enabled=0`、`valid_mask=0`、1 ms 协议占位有效期。参考接收顺序是：解码和完整性/数值校验，
若 `motion_enabled=0` 则立即返回 `FCU_SETPOINT_DISABLED`，只有运动包才按本地接收时刻和
`validity_ms` 区分 `ACTIVE`/`EXPIRED`。因此线上延迟不消耗 TTL，调度延迟也不能把已收到的合法禁用
包降级为可忽略的普通过期包；`fcu_setpoint_is_fresh()` 仅表示“新鲜且允许运动”，不能作为丢弃禁用包
的接收过滤器。`nav_fcu_set_motion_authorized()` 只报告主机最终授权状态，不代表飞控已解锁、接受或
执行。队列拒绝由 `NavApp.fcu_tx_drops` 计数并撤销授权，本地命令构造拒绝由
`fcu_command_rejections` 计数。

## 链路健康状态机

`FcuBridge` 使用固定内存维护：

```text
WAITING → ACTIVE → STALE
                    ↓ 新的有效状态（仅恢复计算）
                  ACTIVE ──→ 等待新的授权决策序号
```

默认 50 ms 没有收到新的飞控状态就进入 `STALE`，导航侧不再生成运动指令。活动链路拒绝
重复和倒序序号，并正确处理 16 位序号回绕；链路已经超时后允许飞控重启并重新同步新的序号
起点。接收飞控的一侧仍必须独立检查指令接收年龄，超过 `validity_ms` 后禁止永久复用
最后一条速度指令。状态链路恢复只允许重新计算，不自动恢复运动授权。具体是刹停、悬停、降落还是停桨，必须在飞控选型后配置并实测；当前
代码和主机测试不证明任何一种实机 Failsafe 已具备。

50/100 ms 是软件回归默认值，不是实机安全结论。它们必须结合状态发送频率、UART 最坏
延迟、飞行速度、制动距离和飞控自身 Offboard/Guided 失联策略标定。

## 参考字节流

参考编码逐字段小端序列化，不直接发送 C 结构体：

| 帧 | 长度 | 内容 |
|---|---:|---|
| FCU 状态 | 100 B | `RMFC`、版本、类型、长度、序号、状态标志、源时间、有效位、能力、姿态、IMU、位置速度、高度、电量、CRC32 |
| 导航指令 | 72 B | `RMFC`、版本、类型、长度、序号、源时间、有效位、最终主控制模式、运动使能、有效期、位置/速度/加速度、偏航/偏航角速度、CRC32 |

参考字节流保持 72 B 和 schema v1：内部 frame 在本版协议中固定为 local XY、Z-up，不单独占线字段；
航向模式由互斥的 yaw/yaw-rate 有效位无歧义恢复。未来若需要传输其他 frame，必须升级 schema，
不能沿用 v1 静默解释。

`FcuLinkParser` 可以从任意分片的 UART 字节流中寻找魔数并在错误长度或噪声后重新同步；完整
帧仍必须经过版本、类型、保留位、CRC 和数值检查。CRC 只能发现传输损坏，不提供身份认证。

## 推荐任务与速率起点

| 数据/任务 | 软件起始值 | 说明 |
|---|---:|---|
| 飞控姿态/角速度/电机环 | ≥500 Hz | 留在飞控，不跨 UART |
| 飞控状态到导航 | 100–200 Hz | 必须带采样时间和序号 |
| 导航到飞控指令 | 50–100 Hz | 飞控侧必须有独立超时 |
| 光流/本地里程计 | 传感器实际频率 | 优先传融合状态，避免重复估计 |
| 电池和模式状态 | 10–20 Hz | 不应占用控制帧优先级 |

建议使用 UART DMA、固定内存收发环和有界的每周期解析预算。460800 baud 可作为起点，链路
稳定时可使用 921600 baud；实际值必须经过线长、电平、干扰和最坏消息负载测试。日志和遥测
如果共用 UART，必须低于状态与控制数据优先级。

## PX4/MAVLink 可选映射

`flight/px4_mavlink/px4_mavlink_adapter.c` 直接复用本文件的两种通用结构：PX4
`LOCAL_POSITION_NED`、`ATTITUDE_QUATERNION` 等消息先归一化为 `FcuStateSnapshot`，再调用
`fcu_state_to_flight_state()`；`NavApp` 产生的唯一 `FcuSetpoint` 在 TTL、模式、frame 和数值检查后
映射为 `SET_POSITION_TARGET_LOCAL_NED`。适配器自身不拥有任务授权，不允许绕过
`FcuMotionAuthorization`。

PX4 没有接收 `RMFC` 禁用包。当前 PC 参考行为是：禁用或过期时不发送运动 setpoint，并在曾经
授权后请求切换到 PX4 Hold（AUTO_LOITER）；链路中断时则无法保证该请求送达，最终动作由 PX4 Offboard-loss 参数决定。
所以 `motion_enabled=0`、停止发送、模式退出 ACK、heartbeat 已退出 Offboard 和实际飞行器动作是
不同证据。详细消息、坐标、时间、依赖版本、运行命令和未验证事项见 [PX4 SITL 最小闭环适配与验证手册](px4_sitl.md)。

## 软件验证

```powershell
.\build\test_fcu_bridge.exe
.\build\test_nav_app_fcu.exe
.\build\test_nav_app_flight_command.exe
.\build\fcu_bridge_sim.exe
```

确定性仿真包含 20 ms 传输延迟、字节流噪声、单帧 CRC 损坏和 300 ms 双向中断。它必须
观察到导航侧状态超时、飞控侧指令过期、恢复后同序号自动低→高仍禁止运动、显式新决策重授权、
禁用包独立于 TTL 的接收语义以及状态序号重新同步，并输出
`FCU_BRIDGE_SIM_SUCCESS`。

## 选型后仍需完成

1. 确认飞控是否原生支持外部速度或加速度目标，以及允许的坐标系和发送频率；
2. 实现对应协议适配器和能力声明，不要修改导航核心的数据结构；
3. 完成上层平台与飞控的时钟偏移估计，或使用飞控协议已有时间同步；
4. 用逻辑分析仪测量采样到发送、线上传输、解析到执行的最坏延迟和抖动；
5. 验证串口拔除、飞控重启、导航重启、序号回绕和队列拥塞；
6. 无桨台架确认飞控不会在指令过期后保持最后速度，再进入系留低速试飞。

当前实现不证明任何具体飞控已经兼容，也不包含 UART 电气、DMA、RTOS 驱动或实机失联
动作。飞控型号和固件版本确定后，必须以其官方外部控制接口为准。
