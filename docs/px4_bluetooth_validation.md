# PX4 蓝牙通信分级验证手册

## 先说结论

当前项目已经有协议无关的 `FcuBridge`、`FcuStateSnapshot` 和 `FcuSetpoint`，并带有 `RMFC` 参考字节流；它们不是 PX4 的 MAVLink 帧。验证 PX4 时需要增加一个适配层，把 PX4 的 MAVLink 消息转换为项目内部结构，再把项目的速度或加速度目标转换为 PX4 支持的 Offboard setpoint。

蓝牙只负责传输字节，不负责导航、坐标转换、失联保护或控制权管理。推荐先使用 Bluetooth Classic 的 SPP/透明串口模块；BLE GATT 不是透明 UART，需要在导航侧额外实现 GATT 主机和特征值协议，不适合作为第一轮验证方案。

PX4 官方将伴随计算机接入定义为可配置串口或以太网链路，消息协议通常使用 MAVLink。因此，以下方案把蓝牙模块看作“无线串口线”，这是基于 PX4 官方接口的工程推论，而不是 PX4 内置了通用蓝牙驱动。[PX4 伴随计算机连接说明](https://docs.px4.io/main/en/companion_computer/pixhawk_companion)和[串口配置说明](https://docs.px4.io/main/en/peripherals/serial_configuration)是配置依据。

## 推荐拓扑

```text
PX4 FCU TELEM2 UART
        │ 3.3 V UART，TX/RX 交叉，GND 共地
        │
Bluetooth Classic SPP 透明串口模块 A
        ))))))  无线链路  ((((((
Bluetooth Classic SPP 透明串口模块 B
        │
导航 MCU UART / USB-UART / PC
        │
PX4 MAVLink 适配器 → FcuStateSnapshot / FcuSetpoint
```

第一轮可以暂时用 PC 代替导航 MCU：PC 通过蓝牙得到一个 COM 口，用 `QGroundControl` 或 `pymavlink` 验证 MAVLink。PC 验证通过后，再把同一端换成导航 MCU 的 UART 驱动。

不要把导航 MCU 的 `RMFC` 帧直接接到 PX4。PX4 不会识别它；需要使用 PX4 支持的 MAVLink 消息，或在 PX4 侧另外开发协议模块。

## 0. 先确定硬件形态

在接线前记录以下信息：

| 项目 | 必须确认 |
|---|---|
| PX4 飞控型号和固件版本 | 不同板卡的 TELEM 端口映射、参数名和默认速率可能不同 |
| 蓝牙类型 | 第一轮选择 Bluetooth Classic SPP/透明串口；不要把 BLE GATT 当作串口 |
| UART 电平 | PX4 串口按 3.3 V 连接，确认蓝牙模块的 IO 电平和供电电压；需要时加电平转换 |
| 最大 UART 速率 | 以蓝牙模块的稳定吞吐为准，先从 115200 开始，不要直接照搬 TELEM2 的 921600 默认值 |
| 导航侧接口 | PC 的 COM 口、USB-UART、STM32 UART 或其他桥接芯片 |
| 安全供电 | 飞控、蓝牙模块和导航 MCU 的电源不能超出额定范围，TX/RX 只交叉连接，GND 共地 |

PX4 官方文档提醒，串口硬件和伴随计算机的电平必须匹配；具体接插件针脚必须以当前飞控板卡手册为准。[伴随计算机串口硬件说明](https://docs.px4.io/main/en/companion_computer/pixhawk_companion)

## 1. 只验证蓝牙透明串口

这一阶段还不接 PX4，目的是把蓝牙问题和飞控问题分开。

1. 两个 SPP 模块配对，固定主从角色和设备地址。
2. PC 连接模块 B，确认出现 COM 口；Linux 下通常会得到一个 RFCOMM 设备。
3. 用串口终端从 A 发送固定字节模式，例如递增序号、`0x55/0xAA` 交替和长帧。
4. 在 B 端回环或接 USB-UART，检查长度、顺序、吞吐和断开重连。
5. 连续运行至少 10 分钟，记录最大延迟、丢帧、乱序和重连耗时。

这一阶段的通过条件是：数据透明、无意外插入字符、断开可检测、重连后可以恢复。通过后再接 PX4。

## 2. PX4 只读链路验证

接线时先拆桨、不要解锁、不要发送运动指令。把蓝牙模块 A 接到 PX4 的 `TELEM2`（或当前板卡指定的伴随计算机串口），TX/RX 交叉，GND 共地。

在 QGroundControl 中配置与当前 PX4 版本对应的 MAVLink 实例。常见的 TELEM2 起点是：

```text
MAV_1_CONFIG = TELEM2
MAV_1_MODE   = Onboard
SER_TEL2_BAUD = 与蓝牙模块两端一致
```

不同 PX4 版本或板卡的实例编号可能不同，应以参数中实际映射到 TELEM2 的 `MAV_X_CONFIG` 为准。修改端口映射后重启 PX4；`MAV_X_RATE` 也要按蓝牙实际吞吐限制调整。[MAVLink 外设配置](https://docs.px4.io/main/en/peripherals/mavlink_peripherals)说明了实例、模式、速率和串口波特率的关系。

先只观察下列消息，不进入 Offboard：

- `HEARTBEAT`：系统是否被发现、系统 ID/组件 ID 是否正确；
- `SYS_STATUS` 或 `BATTERY_STATUS`：电源状态是否可读；
- `ATTITUDE_QUATERNION` 或 `ATTITUDE`：姿态消息是否连续；
- `LOCAL_POSITION_NED` 或 `ODOMETRY`：位置和速度是否连续；
- `HIGHRES_IMU`/`RAW_IMU`：是否有 IMU 数据，频率是否符合预期；
- `COMMAND_ACK`：回传方向是否正常。

不要同时让 QGroundControl、MAVProxy 和自写程序独占同一个 COM 口。需要多个消费者时，在 PC 上增加 MAVLink Router；否则先只使用一个程序，避免把端口占用误认为蓝牙断链。

## 3. 用 PC 做最小 MAVLink 探针

推荐先用 PC 验证返回路径，再实现 STM32 适配器。安装 `pymavlink` 后，下面的最小脚本只等待心跳并打印关键消息，不发送解锁或运动命令：

```python
from pymavlink import mavutil

link = mavutil.mavlink_connection("COM7", baud=115200)
heartbeat = link.wait_heartbeat(timeout=10)
print("connected", heartbeat.get_srcSystem(), heartbeat.get_srcComponent())

for _ in range(100):
    msg = link.recv_match(
        type=["HEARTBEAT", "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED",
              "HIGHRES_IMU", "SYS_STATUS"],
        blocking=True,
        timeout=1.0,
    )
    if msg is not None:
        print(msg.get_type(), msg.to_dict())
```

把 `COM7` 和波特率换成实际值。脚本能收到心跳只证明“蓝牙串口 + PX4 MAVLink 接收方向”可用；还必须检查消息年龄、频率、序号间隙和反向 ACK，才能判定双向链路。

建议记录以下指标：

| 指标 | 记录方式 | 目标如何确定 |
|---|---|---|
| 心跳周期和抖动 | 接收时间戳差分 | 根据蓝牙吞吐和任务周期实测 |
| 最新状态年龄 | 本地接收时刻减消息时间 | 必须小于导航状态超时 |
| 丢帧/乱序 | MAVLink 序号、消息计数 | 连续运行和干扰场景统计 |
| 往返延迟 | 请求/ACK 或 PING 时间差 | 取最大值和高分位数 |
| 断链检测 | 拔电、离开范围、关闭模块 | 必须触发桥接层和 PX4 侧失联动作 |

## 4. 映射到当前项目的 `FcuStateSnapshot`

PC 探针通过后，再实现 PX4 MAVLink 适配器。适配器只做协议、坐标、时间和能力映射，不修改 `NavRuntime`。

| PX4/MAVLink 数据 | `FcuStateSnapshot` | 注意事项 |
|---|---|---|
| `ATTITUDE_QUATERNION` 或 `ODOMETRY` | `attitude` | 确认四元数顺序和参考方向，不能仅按字段名字复制 |
| `ATTITUDE`/`ODOMETRY` | `angular_velocity` | 确认机体系、单位和轴方向 |
| `HIGHRES_IMU`/`RAW_IMU` | `linear_acceleration` | 使用比力语义和 m/s²；确认是否需要轴变换 |
| `LOCAL_POSITION_NED`/`ODOMETRY` | `local_position`、`local_velocity` | PX4 常用 NED，项目约定 z 向上，至少要做 `z_nav = -z_ned` |
| `DISTANCE_SENSOR` 或高度字段 | `height_m` | 只在传感器有效时设置有效位 |
| `HEARTBEAT`、估计器状态 | `armed`、`external_control_active`、`estimator_healthy` | 使用状态位，不要根据“收到心跳”假设估计器健康 |
| `BATTERY_STATUS` | `battery_remaining` | 未知电量必须清除有效位 |

`source_timestamp_ms` 应保存飞控采样时间；链路超时则使用导航侧本地接收时间。项目内部使用导航坐标系 z-up，而 PX4 常用 NED，坐标变换必须集中在适配器中。首次验证时，把静止飞控的 MAVLink 数据、转换后的 `FcuStateSnapshot` 和 QGroundControl 显示值并排记录，先确认静止、倾斜和手动平移三种情况。

## 5. 映射 `FcuSetpoint` 到 PX4 Offboard

第一种控制量建议只做“速度目标 + 偏航”，不要一开始同时发送位置、速度和加速度。适配器可以把 `FcuSetpoint.velocity_sp` 映射为 `SET_POSITION_TARGET_LOCAL_NED` 的 velocity 字段，并设置其他字段的 type mask 为忽略。

PX4 Offboard 要求外部控制器持续发送至少 2 Hz 的生命信号/支持的 setpoint，且进入 Offboard 前就应已开始发送；超过 `COM_OF_LOSS_T` 未收到消息后，PX4 会退出 Offboard 并执行配置的失联动作。[PX4 Offboard 官方说明](https://docs.px4.io/main/en/flight_modes/offboard)

因此项目侧应同时满足：

1. 导航 MCU 的 `nav_fcu_setpoint_write` 非阻塞，只复制到固定容量发送队列；
2. 适配器按 PX4 支持的字段发送，不重复闭合速度环和加速度环；
3. 每条命令带序号、生成时间和有效期；
4. 蓝牙断链时停止沿用最后一条速度命令；
5. PX4 自己的 Offboard 失联保护仍然开启，由飞控最终执行降级动作。

当前项目默认 `FcuSetpoint` 有效期和 `RMFC` 帧只用于项目内部回归。PX4 适配器完成后，必须重新测量从导航生成 setpoint 到 PX4 接收和执行的最坏延迟，不能把软件仿真中的 50/100 ms 直接当成飞行参数。

## 6. 分级验收顺序

```text
S0  蓝牙透明串口回环
 ↓
S1  PX4 心跳和只读遥测
 ↓
S2  双向 MAVLink ACK/PING，无运动
 ↓
S3  静止/手动移动状态映射与坐标检查
 ↓
S4  拆桨、未解锁，发送 Offboard setpoint 但不进入飞行
 ↓
S5  拆桨/系留条件下主动断链，验证 PX4 失联动作
 ↓
S6  SITL/HIL 重复 S3～S5
 ↓
S7  系留低速实飞，最后才逐步开放运动范围
```

每一级都保存：PX4 参数、固件版本、蓝牙模块地址和波特率、原始 MAVLink 日志、导航日志、断链时间、恢复时间和最终安全状态。S0～S3 通过后，才值得把适配器接入当前 `NavRuntime`；S5 通过前不要在空中启用 Offboard。

## 7. 安全与协议边界

- 蓝牙链路不是姿态环链路。姿态、角速度、混控和电机输出必须留在 PX4 内部。
- MAVLink 默认不提供身份认证；实验环境也不要把未签名链路暴露到陌生设备，正式系统应评估 MAVLink signing 和蓝牙配对安全。[PX4 MAVLink 安全说明](https://docs.px4.io/main/en/mavlink/)
- CRC 或 MAVLink 校验只能发现传输损坏，不能证明发送者可信。
- 所有 Offboard 测试先拆桨，随后使用系留和人工急停；不要以“收到心跳”作为可解锁条件。
- 蓝牙的无线重传、休眠、共存干扰和距离变化会带来抖动；最终是否能用于飞行，必须以最坏延迟和失联实测决定。

## 当前项目的下一步实现边界

硬件型号和 PX4 版本确定后，再新增一个具体适配器，例如：

```text
platform/stm32/px4_mavlink_adapter.c/.h
    MAVLink 接收 → FcuStateSnapshot
    FcuSetpoint → SET_POSITION_TARGET_LOCAL_NED
    nav_fcu_setpoint_write → 固定内存 UART/蓝牙 TX 队列
```

这个适配器应作为飞控/板级代码存在，不应把 MAVLink 字段直接塞进 `core/`。现有 `FcuBridge`、状态超时、指令有效期和主机回归可以继续复用。
