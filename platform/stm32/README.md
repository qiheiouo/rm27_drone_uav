# STM32 移植层

本目录提供板级接口契约和固定内存 `NavApp` 装配。`NavApp` 内部调用与主机仿真相同的 `NavRuntime`，本目录只读取板级输入并发送控制输出。它能在主机编译检查，但不包含具体 HAL、RTOS、相机检测器或低层飞控。

## 任务优先级

```text
最高：IMU 采样、姿态/角速度环、混控和电机输出（建议 ≥500 Hz）
中等：nav_app_step（建议 100–200 Hz）
较低：视觉前端、通信、日志和遥测
```

导航层失效时，低层仍必须保持改平、限幅和受控降落能力。`nav_fcu_setpoint_write` 传递带序号和有效期的协议无关指令包；飞控适配器优先使用速度目标，只有飞控明确支持时才使用加速度目标。

## 板级接口

`nav_platform.h` 要求目标工程提供：

- 单调毫秒时钟；
- IMU、ToF、光流、前视目标像素和下视基座像素；
- 固定容量局部障碍列表；
- 任务开始、返航、急停命令；
- 停靠接触、充电和无线充电就绪信号；
- 他机状态接收和本机状态广播；
- FCU 指令、解锁状态、低频日志，以及非阻塞遥测队列出口。

除 IMU 外，其余输入允许本帧无效。板级实现必须使用统一坐标系、SI 单位和单调时间戳，并处理 32 位毫秒计数回绕。

共享运行时会拒绝过期、重复、倒序或数值无效的输入，并监测导航任务周期。IMU 异常会使本周期控制输出失效；可选传感器异常则丢弃该通道并继续降级运行。连续周期超时会锁存 watchdog 并请求任务安全覆盖。`nav_log` 接收固定内存事件环中的新增事件名称，板级实现必须保持非阻塞。

`nav_telemetry_write` 接收完整的 152 字节遥测帧。板级实现必须在函数返回前复制到低优先级队列；队列满时立即返回负值，不能在导航任务中等待串口、USB、CAN 或存储设备。帧序号在队列拒绝前已经递增，因此接收端可通过序号间隙统计丢帧。默认每 10 次 `nav_app_step` 输出一帧，可在编译时覆盖 `NAV_APP_TELEMETRY_PERIOD_STEPS`；定义为 `0` 可完全关闭。

## 双 MCU 飞控适配

飞控选型未确定时，板级代码只依赖 `FcuStateSnapshot` 和 `FcuSetpoint`，不让
`NavRuntime` 直接包含 MAVLink 或具体厂商消息。飞控 UART 接收任务应当：

1. 使用飞控原生协议或参考 `RMFC` 字节流解码状态；
2. 转换坐标、单位和采样时间后调用 `fcu_bridge_ingest_state`；
3. 从活动快照生成 `ImuSample`、`OdomSample` 和高度缓存，供现有读取接口非阻塞获取；
4. 超过状态时限后停止向导航提供新 IMU，触发既有输入拒绝路径。

`nav_fcu_setpoint_write` 必须在返回前复制完整 `FcuSetpoint` 到高优先级 TX 队列。协议
适配器根据飞控能力选择速度或加速度字段，转换导航 z-up 坐标到飞控坐标，并映射
`motion_enabled`、有效期和外部控制失联动作。它不得同时重复闭合速度和加速度控制环。
队列拒绝通过 `NavApp.fcu_tx_drops` 统计。完整约定和故障仿真见
[双 MCU 飞控 UART 桥接](../../docs/fcu_uart_bridge.md)。

## 使用方式

```c
static NavApp app;
static NavAppConfig config;

void nav_task(void *argument)
{
    nav_runtime_config_default(&config);
    board_apply_calibrated_nav_config(&config);
    if (nav_app_init(&app, &config) != NAV_CONFIG_ERROR_NONE) {
        board_enter_safe_fault();
    }
    for (;;) {
        nav_app_step(&app, 0.01f);
        os_delay_until_next_period();
    }
}
```

`nav_app_init` 会检查任务时序、安全截止时间、估计器、控制器、规划工作区、相机、航线与蜂群参数，失败时保持未解锁。该检查只能排除结构性错误，配置值仍必须来自实机标定和安全评审。`simulation/scenario.c` 不能当作可直接飞行的参数集。

## 约束检查

- C99、`float`、固定容量数组和静态实例；无堆分配、异常、RTTI 或动态容器。
- `nav_core` 与 `nav_stm32_port` 可独立于 `nav_sim` 构建；固件不得链接 `simulation/`。
- 光流工作区、轨迹段、障碍和他机数量都有编译期上限。
- 规划器迭代、轨迹采样和预测时间窗均有配置上限。
- 在目标编译器上启用警告、栈使用报告和链接 map；测量 `sizeof(NavApp)`、任务栈峰值和最坏 tick 时间。

本仓库没有在具体 STM32 型号上给出已测 RAM 或周期数字。任何性能估算都必须以目标 MCU、编译选项、FPU、缓存和真实传感器负载下的测量为准。

## 上机闸门

1. 坐标轴、重力方向、四元数顺序、相机安装矩阵和单位通过台架测试；
2. IMU/ToF/视觉/通信逐项断连时状态机符合预期；
3. 急停与硬围栏不受恢复流程屏蔽；
4. 电机未装桨条件下验证解锁、指令限幅和 watchdog；
5. SIL/HIL 覆盖撞击、目标丢失、基座丢失、障碍和双机消息超时；
6. 系留低速飞行后再开放末端接触与自动停靠。
7. 逻辑分析仪确认 FCU 状态和导航指令的最坏延迟、抖动与断线超时符合标定值。
