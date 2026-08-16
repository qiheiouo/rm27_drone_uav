# RM_Drone_Nav

RoboMaster 微型无人机自主导航（Plan B：完全机载自主）。
总体设计约束与方案论证见 `General_Plan.md`，架构细节见 `docs/architecture.md`。

在 STM32 级计算资源约束下，实现：短时自主出航 → 目标视觉接近 → 接触目标
→ 撞击/翻倾恢复 → 定位恢复 → 返航 → 基座视觉精确停靠。
单机串行任务，但保留多机并行扩展接口（`swarm/`）。

## 当前状态

host 仿真 + MCU 兼容核心，完整任务闭环在**真实算法链路**下验证通过：

| 子系统 | 实现 |
|---|---|
| 状态估计 | INS（陀螺积分 + Mahony 姿态修正）+ 光流 VO 水平校正 + ToF 高度融合 + 健康状态机（TRACKING/DEGRADED/LOST/RECOVERING/RELOCALIZED） |
| 视觉里程计 | 下视光流：id 关联 → 陀螺旋转补偿 → 平移最小二乘 → 外点剔除（尺度来自 ToF） |
| 目标/基座检测 | 针孔相机模型 + 像素观测 + 已知尺寸重建相对位置（PnP-lite） |
| 轨迹 | 航点间 minimum-jerk 五次多项式（MINCO-lite），位置+速度前馈跟踪 |
| 任务管理 | 17 态 FSM，覆盖目标丢失/估计器丢失/超时返航/围栏/紧急降落 |
| 撞击恢复 | 比力偏离 1g + 陀螺尖峰检测；翻倾分级处理（>20° 只保高度）；三条件退出 |
| 基座停靠 | 螺旋搜索 → marker 重定位 → 视觉伺服下降；视觉对准 + 机械捕获双判据 + 低空盲降兜底 |

验证：40 项 ctest（7 单元 + 闭环集成 + 32 组蒙特卡洛）全过；
55 组手动扫掠（正常 30 / 撞击 LOST 15 / 剧烈翻倾 10）全过。
任务耗时 avg ~19 s / max ~24 s（规则预算 30 s），停靠误差 ≤0.13 m。

## 构建与运行

依赖：CMake + 任一 C99 编译器（Windows 已验证 clang + Ninja；Linux/macOS 用 gcc/clang 均可）。

```sh
cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=clang   # 配置
cmake --build build                                     # 构建
cd build && ctest --output-on-failure                   # 全部测试
```

### 仿真模式

```sh
./build/mission_sim.exe                       # 默认：INS + 光流 VO + ToF 全链路
./build/mission_sim.exe --seed 7              # 换随机种子（偏置/漂移/噪声/特征分布）
./build/mission_sim.exe --lost-on-impact      # 最坏情况：撞击后估计器直接 LOST
./build/mission_sim.exe --hard-impact         # 剧烈撞击：>90° 翻倾恢复
./build/mission_sim.exe --sim-vo              # 对照：合成 VO（注入式漂移模型）
./build/mission_sim.exe --truth-mode          # 对照：理想真值透传（算法上限参考）
```

退出码：`0` 任务完成（DOCKED）；`2` 紧急降落；`3` 仿真超时。
场景参数（场地/航点/目标位置/各模块增益）集中在 `simulation/scenario.c`，
改场景只动这一个文件。

## 仓库结构

```
core/            MCU 兼容核心算法（纯函数，无任何平台依赖）
  math/            Vec3f + 四元数（header-only）
  filters/         一阶低通
  state_estimator/ INS + Mahony + VO/ToF 融合 + 健康状态机
  mission/         任务 FSM
  guidance/        轨迹跟踪/末端/恢复/基座/紧急降落制导律
  planner/         航点队列 + min-jerk 多项式轨迹
  safety/          撞击检测、时间预算、地理围栏
perception/      机载感知算法
  vision_frontend/ 光流里程计
  camera/          针孔模型 + 像素→相对位置重建
  target_tracker/  目标跟踪    home_detector/  基座 marker 跟踪
flight/          导航↔飞控分层接口（GuidanceOutput → CtrlOutput）
swarm/           多机扩展接口（第一版 other_count=0）
simulation/      仅仿真侧：动力学、IMU/ToF/相机/特征仿真、场景配置
platform/
  host/          仿真主程序
  stm32/         移植层：板级接口契约 + 固件装配参考
tests/           单元测试
docs/            架构文档
```

**核心边界**：`core/`、`perception/`、`flight/`、`swarm/` 只依赖 `core/math`，
不含任何平台/仿真代码，可直接编译进固件；`simulation/` 与 `platform/host/`
仅用于 host 验证，不上机。

## 对接方法

### 1. 对接到 STM32 固件

core 全部是 `xxx_update(state*, input*, output*)` 纯函数，
C99、float32、固定容量数组、静态内存，无 malloc/STL/异常/RTTI。

最小集成路径：

1. 把 `core/`、`perception/`、`flight/`、`swarm/` 加入固件工程
  （include 路径参考 `CMakeLists.txt` 的 `target_include_directories`）。
2. 实现 `platform/stm32/nav_platform.h` 声明的 9 个板级函数
  （IMU/ToF/光流/双相机读取、FCU 指令下发、解锁、日志、时钟）。
3. 用 `platform/stm32/nav_tasks.c` 的 `NavApp` 装配：
   ```c
   static NavApp app;
   static NavAppConfig cfg;   /* 参数参考 simulation/scenario.c 的默认值 */
   nav_app_init(&app, &cfg);
   /* RTOS 任务，100~200 Hz 周期调用 */
   nav_app_step(&app, 0.01f);
   ```
   `nav_app_step` 与 host 仿真 `main.c` 保持相同的模块调用顺序，
   在仿真里验证的行为即在固件里的行为。
4. 低层飞控（姿态/角速度环、混控、电调）不在本仓库范围：
   复用既有固件（`nav_fcu_send` 映射到其外部控制接口），或按
   `platform/stm32/README.md` 的任务布局自行实现。**铁律：姿态稳定
   不依赖导航层**——导航层失效时低层必须保持悬停/改平能力。

### 2. 对接真实传感器

替换关系（接口已在仿真中验证，逐一替换即可）：

| 仿真件 | 真实件 | 对接点 |
|---|---|---|
| `sim_sensors_imu` | BMI088 等 IMU 驱动 | `nav_imu_read` → `ImuSample`（机体系比力+陀螺） |
| `sim_sensors_tof` | ToF 测距模组 | `nav_tof_read` → 高度（m） |
| `sim_vision_frame` | 光流模组（PMW3901 类）或片上特征前端 | `nav_flow_read` → `FlowFrame`（id 关联的特征像素） |
| `sim_sensors_target` | 前视相机 blob/目标检测器 | `nav_camera_target_read` → `PixelObs`（u,v,size_px） |
| `sim_sensors_home` | 下视相机 marker 检测器（AprilTag/自定义图案） | `nav_camera_home_read` → `PixelObs` |

注意：`PixelObs` 只需提供目标中心的像素坐标与表观像素尺寸，
相对位置重建（`camera_reconstruct_nav`）由 core 完成，
检测器本身可以是任意图像算法。

### 3. 对接真实场地/任务参数

- 基座位置、出航/搜索航线、目标大致区域：`NavAppConfig.home_pos` /
  `outbound_route` / `search_route`（仿真侧对应 `scenario.c`）。
- 目标与 marker 的真实尺寸：`target_size_m` / `marker_size_m`
  （测距依赖已知尺寸，标定后填入）。
- 相机内参与安装方向：`CameraModel`（`CAM_MOUNT_FORWARD`/`CAM_MOUNT_DOWN`
  或自定义 3x3 安装矩阵）。
- 任务预算/围栏/各超时：`MissionConfig` + `SafetyConfig`。

### 4. 扩展到多机并行（未来）

`swarm/` 已保留接口：`SwarmView.others[]` 填入他机状态，
`TrajectoryMessage` 广播本机未来轨迹，`collision_check` 做冲突检测。
当前 `other_count=0`，单机逻辑不为此支付任何成本。

## 文档

- `General_Plan.md` — 总体方案与约束（Plan A/B/C、三层定位、任务闭环）
- `docs/architecture.md` — 模块划分、关键设计决策（含调试教训）、验证结果
- `platform/stm32/README.md` — 移植任务布局、约束检查单、资源估算
