# PX4 SITL 最小闭环适配与验证手册

> 状态（2026-10-16）：PX4/MAVLink 适配代码为 `Implemented / Sim-verified（主机离线协议与组合路径）`；真实 PX4 SITL 会话为 `未验证`。本机没有 WSL Linux 发行版、Docker 或 PX4-Autopilot 源码，因此本阶段总判定为 `Modify`，不得把本文中的运行入口或 Mock/离线消息测试表述为 SITL 已飞行。

## 1. 选型与边界

飞控型号已核对为 **MicoAir743v2-AIO-35A**（STM32H743VIH6、35 A 四合一电调），厂商给出的 PX4 target 是 `micoair_h743-v2`，并说明从 PX4 1.16.0 起进入官方支持。机上实际固件版本、构建哈希、参数和端口配置尚未读取；PX4 文档当前稳定版本也不能代替实机版本证据。

本阶段选择 MAVLink common 消息和官方生成的 C 头文件，而不是 MAVSDK、ROS 2 或自定义 `RMFC`：

- 协议层继续使用 C99、固定内存，不依赖宿主线程模型；
- PC 运行入口只在 `platform/host/` 使用 UDP；未来串口、ARM Linux 或 MCU 传输只需替换收发层；
- `RMFC` 保留为已有参考字节流，不与 MAVLink 并列成为 PX4 的正式运动输出；
- 不修改 FlightState、FlightCommand、任务状态机、规划器、视觉和末端制导公共契约。

第三方依赖记录：

| 项目 | 记录 |
|---|---|
| 来源 | `https://github.com/mavlink/c_library_v2.git` |
| 验证版本 | commit `8c63a49fe35aead7f9ef14031381c5e076ff743a`（由 mavlink definitions `2e2b6af99084e4fde9bb304c84bb9253a1a98b6f` 生成） |
| 许可证 | 官方 MAVLink 开发文档声明生成的 C 头文件为 MIT；生成器本身为 LGPLv3 |
| 引入方式 | 不 vendoring；通过 `MAVLINK_INCLUDE_DIR` 指向独立 checkout，仅在 `NAV_ENABLE_PX4_MAVLINK=ON` 时参与构建 |
| 本地修改 | 无 |
| 移除方式 | 关闭 CMake 选项并删除本地 `build/deps/c_library_v2`；默认 84 项构建与测试不依赖它 |

## 2. 唯一数据与控制路径

状态路径：

```text
PX4 MAVLink UDP 字节流
  → 官方 mavlink_parse_char
  → px4_mavlink_ingest
  → FcuStateSnapshot（已完成 NED/FRD → 导航 z-up/FLU）
  → fcu_state_to_flight_state
  → nav_flight_state_read
  → NavApp / NavRuntimeConfig.flight_state_source=EXTERNAL
  → NavState（跳过内部惯性估计器）
```

控制路径：

```text
NavRuntime / CtrlOutput.velocity_cmd
  → NavApp::FcuMotionAuthorization（唯一最终主机授权）
  → FcuSetpoint（velocity + yaw 或 yaw-rate、local z-up、TTL）
  → nav_fcu_setpoint_write
  → px4_mavlink_make_velocity_setpoint
  → SET_POSITION_TARGET_LOCAL_NED
  → PX4 Offboard 接收端
```

适配器不生产第二条任务级速度，不递增 `motion_authorization_sequence`。运行器只把命令行中的一次明确授权决定在任务首次进入运动状态时提交；状态失效撤销后不会自动制造新序号。模式切换和解锁请求是 PX4 控制生命周期命令，不是另一条速度链。

## 3. 状态输入契约

适配器消费：

- `HEARTBEAT`：确认 `MAV_AUTOPILOT_PX4`，锁定首个匹配的 system ID（或配置的固定 ID），读取 armed 位、`custom_mode` 主模式和系统状态；同一进程不自动切换到第二个 PX4；
- `LOCAL_POSITION_NED`：PX4 EKF 局部位置与速度；
- `ATTITUDE_QUATERNION`：WXYZ、FRD body 到 NED earth 的姿态及体轴角速度；
- `HIGHRES_IMU`：仅在 X/Y/Z 加速度更新位齐全且数值有限时作为碰撞检测所需原始比力；
- `ESTIMATOR_STATUS`：一旦收到，就要求持续新鲜且姿态、水平/垂直速度、水平位置、高度有效；
- `EXTENDED_SYS_STATE`：记录 landed state；
- `COMMAND_ACK`：记录最近命令和结果，不能把“已发送”冒充“已接受”。

默认健康门限为 heartbeat 1500 ms、融合状态 200 ms、IMU 100 ms。状态有效还要求 heartbeat system status 为 `STANDBY` 或 `ACTIVE`。PX4 boot time 回退会按重启处理，清除已经拼装的状态，等待位置、姿态和 IMU 分别重新到达。非 PX4 heartbeat、错误 system ID、非有限向量、畸形四元数和缺少更新位的 IMU 均拒绝。

`FlightState.timestamp_ms` 使用导航主机的本地单调接收时间，取位置和姿态两项中较旧者，用于实际超时判断；PX4 `time_boot_ms` 保留在 `FcuStateSnapshot.source_timestamp_ms` 作来源诊断。当前没有 TIMESYNC，因此不伪装成已完成跨机时钟同步。

## 4. 坐标和航向

本阶段固定的 **SITL 配置** 为：

- PX4 local NED：+X 北、+Y 东、+Z 下；PX4 body 为 FRD；
- 导航 local z-up：+X 东、+Y 北、+Z 上；内部 body 为 FLU；
- `nav = (ned.y, ned.x, -ned.z)`；角速度和 IMU 由 FRD 转为 `(x, -y, -z)`；
- PX4 WXYZ `FRD→NED` 四元数与两个固定旋转复合，得到内部 WXYZ `FLU→nav`；
- 导航 yaw=0 指向东，对应 PX4 NED yaw=+π/2；z 轴 yaw-rate 符号反向。

导航原点直接等同 PX4 EKF local origin。这个选择只用于当前 SITL；它不证明真实场地东/北、起飞点、机头零向和任务 +X 已对齐。实机前必须标定场地到 PX4 local frame 的平移/偏航关系，不能只依据“都是 z-up”省略水平对齐。

## 5. 控制、Offboard 与失效

已授权速度包映射为 `SET_POSITION_TARGET_LOCAL_NED`：位置和加速度位忽略，速度三轴有效，yaw 与 yaw-rate 二选一。消息时间由最新 PX4 boot time 加本地接收后的经过时间估算，避免直接把 PC 启动时钟填成 PX4 boot clock。NaN/Inf、过期、错误 frame 或模式组合在编码前拒绝。

主机运行器按 100 Hz 调用 NavApp，并采用以下顺序：

1. 连续发送有效 setpoint 至少 1.1 s；
2. 请求 PX4 `OFFBOARD`，并从 heartbeat 确认主模式；
3. 只有显式给出 `--arm` 时才发送解锁请求；
4. 分别记录主机授权、setpoint 发送、Offboard heartbeat 和 armed heartbeat；
5. 授权撤销时停止发送运动 setpoint，并请求 `POSCTL` 退出 Offboard；链路已断时只能依赖以后实测和配置的 PX4 Offboard-loss 行为。

官方要求 MAVLink Offboard setpoint 持续高于 2 Hz，并在切入前持续发送至少 1 s；失流超过 `COM_OF_LOSS_T` 后的动作由 PX4 与 `COM_OBL_RC_ACT` 等参数决定。本代码的“禁用”表示主机协议层不再授权/发送运动目标，并主动请求退出模式；它不等于 PX4 已悬停、降落、停桨或执行 Failsafe。`COMMAND_ACK` 也只证明命令处理结果，不证明飞行器已经达到目标状态。

## 6. 构建和运行

默认构建不需要 MAVLink。可选构建示例：

```powershell
git clone https://github.com/mavlink/c_library_v2.git build/deps/c_library_v2
git -C build/deps/c_library_v2 checkout 8c63a49fe35aead7f9ef14031381c5e076ff743a
cmake -S . -B build/px4 -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DNAV_WARNINGS_AS_ERRORS=ON -DNAV_ENABLE_PX4_MAVLINK=ON -DMAVLINK_INCLUDE_DIR="$PWD/build/deps/c_library_v2"
cmake --build build/px4 --parallel 4
ctest --test-dir build/px4 --output-on-failure
```

运行器硬编码只绑定/接受 IPv4 loopback，且必须同时给出三个显式安全开关；它不能用于实体飞控：

```powershell
.\build\px4\px4_sitl_runner.exe --sitl --authorize-motion --arm --duration-s 30 --log build\px4\px4_sitl.csv
```

PX4 SITL 应把 Offboard MAVLink UDP 数据发到 `127.0.0.1:14540`；端口被占用时可用 `--port`，并同步修改 SITL 端。运行器配置低速场景：起飞到 1.0 m，以不高于 0.5 m/s 前往导航坐标 `(1,0,1)`，随后在该点保持；不含目标观测或撞击动作。退出或授权丢失时发送一次/重试 POSCTL 请求，但必须从 heartbeat、ACK 和飞行日志确认 PX4 的真实结果。

CSV 以 100 Hz 记录状态年龄/采样时刻、主机授权、setpoint 计数、Offboard/armed 反馈、ACK、位置、实际速度、命令速度、航点误差和任务状态。状态接收频率可由 `state_sample_ms` 的变化计算，控制频率可由 `setpoints_sent` 计算，命令—速度响应延迟可从同一日志离线估计。

## 7. 已验证与未验证

主机离线测试已覆盖：

- NED→z-up、FRD→FLU、WXYZ 姿态、yaw/yaw-rate 符号；
- 非有限位置、畸形四元数、状态/IMU/heartbeat 过期、估计健康失效与 PX4 boot time 回退；
- velocity+yaw/yaw-rate type mask、命令 TTL、模式不匹配、禁用状态和 MAVLink 线格式解析；
- Offboard/POSCTL/arm/消息间隔命令及拒绝 ACK 记录；
- `PX4消息→FlightState→NavApp→授权门→FcuSetpoint→MAVLink` 组合路径；
- 定位失效撤权、恢复但旧授权序号仍无运动、新授权序号后非目标航点任务恢复；
- 原有 84 项默认 CTest 与 SIL 不依赖 MAVLink，构建默认行为不变。

尚未验证：

- PX4 SITL 是否接受本运行器的模式/解锁请求并实际起飞、悬停和到达航点；
- 实际状态/控制频率、跟踪误差、响应延迟、ACK 顺序、模式拒绝、通信中断和 Offboard-loss 行为；
- `ESTIMATOR_STATUS` 与 `HIGHRES_IMU` 在目标 PX4 版本/stream 配置中的可用性；
- 实机串口/USB、电平、波特率、MAVLink signing、时钟同步、飞控参数和 Failsafe；
- MicoAir743v2-AIO-35A 上的实际固件版本、板级端口映射和无桨/HIL/飞行行为。

## 8. 阶段判定和后续门槛

| 项目 | 判定 | 证据/缺口 |
|---|---|---|
| MAVLink C99 状态/指令适配 | `Go（L1）` | 严格警告构建、官方消息线格式和组合路径测试通过 |
| PC loopback SITL 运行入口 | `Implemented / 未验证` | 可编译且安全参数/loopback 门生效；没有 PX4 进程可连接 |
| PX4 SITL 最小飞行闭环 | `Modify` | 本机缺少 WSL/PX4/Docker，未产生 PX4 日志或飞行轨迹 |
| HIL/实体飞控 | `Stop` | 本阶段明确禁止；需先完成 SITL、参数审查和无桨台架 |

下一轮进入 TargetObservation 前，建议先保留本适配器的固定坐标/时间配置，取得一次真实 PX4 SITL 低速闭环日志，并确认视觉输出的坐标系能够转换到同一导航 local frame、采样时间可映射到主机单调时钟、丢失语义明确。固定目标真值只能作为 SITL 场景配置，不能冒充已经完成外部视觉协议。

## 9. 官方依据

- [PX4 Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [PX4 Windows/WSL 开发环境](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl.html)
- [MAVLink Common Message Set](https://mavlink.io/en/messages/common.html)
- [MAVLink C generated headers](https://github.com/mavlink/c_library_v2)
- [MicoAir743v2-AIO-35A 官方手册](https://micoair.cn/en/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)
