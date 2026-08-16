# platform/stm32 — STM32 移植层

本层是把 MCU 兼容 core 跑到真实硬件上的装配参考。**当前为骨架**：
接口契约 + 固件主循环参考实现 + 移植清单；不含具体 HAL 驱动
（UART/SPI/I2C/DMA 等由目标板工程实现 `nav_platform.h` 中的函数）。

## 分层与任务布局（Plan 第 9 节）

```
优先级最高 ─────────────────────────────────
  IMU 采样任务        ≥500 Hz   陀螺/加速度计读取（本仓库接口之外）
  姿态/角速度环       ≥500 Hz   低层飞控（本仓库不提供，见下）
  电机混控输出        ≥500 Hz
─────────────────────────────────────────
  导航环 nav_app_step 100~200 Hz  估计器/FSM/制导/位置控制
  视觉前端            相机帧率     特征提取（或外部光流模组）
  日志/遥测           1~10 Hz
优先级最低 ─────────────────────────────────
```

铁律：**姿态稳定不依赖导航层**。视觉挂了、规划挂了、定位挂了，
低层姿态环必须继续工作。导航层只通过 `nav_fcu_send(CtrlOutput)`
下发期望加速度/偏航角速度，绝不直接控制电机。

## 低层飞控

本仓库范围是导航层。低层（姿态/角速度环 + 混控 + 电调）有两个来源选项：

1. 复用既有固件（Betaflight 类），`nav_fcu_send` 映射到其外部控制接口；
2. 后续自行实现级联 PID（姿态环→角速度环→混控），应放在 `flight/` 下
   并保持与导航层的 `CtrlOutput` 接口不变。

## 板级待实现接口（nav_platform.h）

| 函数 | 说明 | 无效返回 |
|---|---|---|
| `nav_time_ms` | 单调毫秒时钟 | — |
| `nav_imu_read` | 机体系比力+陀螺（硬依赖，无效则跳过本 tick） | <0 |
| `nav_tof_read` | 机体 -z 方向 ToF 测距 | <0 |
| `nav_flow_read` | 光流特征帧（建议外部 PMW3901 类模组或片上前端） | <0 |
| `nav_camera_target_read` | 前视目标像素观测 | <0 |
| `nav_camera_home_read` | 下视基座 marker 像素观测 | <0 |
| `nav_fcu_send` | 加速度/偏航角速度指令下发 | — |
| `nav_fcu_set_armed` | 解锁/断油 | — |
| `nav_log` | 日志 | — |

## 移植约束检查单（Plan 第 12 节）

- C99 / C++11-14 子集；`float32`；固定容量数组；静态内存
- 禁止：malloc/free 高频使用、STL 动态容器、异常、RTTI、ROS/OpenCV/Eigen 动态矩阵
- core/ 与 perception/ 全部为 `xxx_update(state*, input*, output*)` 纯函数，
  host 仿真与固件复用同一套代码（`nav_app_step` 与 host `main.c` 调用顺序一致）
- CMSIS-DSP 可用于后续滤波/矩阵加速，但当前代码无此依赖

## 资源估算（基于 host 侧结构体尺寸）

- 全部模块实例（NavApp）约数 KB RAM（静态）
- 视觉前端 48 特征 × 约 24 B 工作区 ≈ 5 KB
- 轨迹 8 段 × 3 轴 × 6 系数 ≈ 0.6 KB
- 导航环单步计算量：光流 LS(3x3 闭式) + 互补滤波 + FSM + 多项式求值，
  Cortex-M7 @480MHz 下预期远低于 1 ms，100~200 Hz 裕量充足

## 使用

```c
static NavApp app;            /* 静态分配 */
static NavAppConfig cfg;      /* 由板级配置填充（参考 simulation/scenario.c） */

void nav_task(void *arg)      /* RTOS 任务，100~200 Hz */
{
    nav_app_init(&app, &cfg);
    for (;;) {
        nav_app_step(&app, 0.01f);
        os_delay_until_next_period();
    }
}
```
