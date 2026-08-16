# 架构说明（第二阶段）

设计依据：`General_Plan.md`。核心原则——不是把高飞组系统缩小，
而是重新设计一套 STM32 能承担的最小自主闭环。

## 数据流（第二阶段：真实传感链路）

```
动力学真值（点质量 + 推力矢量姿态模型 + 姿态环带宽限制）
   ├─→ IMU：机体系比力(含重力) + 陀螺 + 固定偏置 + 噪声
   ├─→ VO：真值 + 常值漂移 + 噪声 + 撞击 blackout
   └─→ 相机：针孔投影 → 像素观测（噪声/量化/视锥剔除）
                │
IMU ─→ INS 估计器（姿态积分 + Mahony 修正 + VO 互补校正 + 健康机）
                │                    │
                │     像素 + 已知尺寸 + 估计姿态 → 相对位置重建（PnP-lite）
                │                    │
                ↓                    ↓
        Impact Detector      Target / Home Tracker
                │                    │
                └──────────┬─────────┘
                           ↓
        Mission FSM ← Safety（时间预算/围栏）
                           ↓
        Cruise / Terminal / Recovery / Home Guidance
                           ↓
        GuidanceOutput → Pos Controller → 动力学
```

第三层定位：marker 捕获上升沿 → `estimator_notify_relocalized`
注入绝对参考，消除 VO 累计漂移；marker 不可见时螺旋搜索扩张覆盖漂移偏差。

## 模块划分

| 目录 | 职责 | 状态 |
|---|---|---|
| `core/math` | Vec3f + 四元数（陀螺积分/旋转/推力矢量姿态构造） | ✅ |
| `core/filters` | 一阶低通 | ✅ |
| `core/state_estimator` | INS 预测 + Mahony + VO 互补校正 + 健康状态机；TRUTH 透传对照模式 | ✅ |
| `core/mission` | 任务 FSM（17 个状态，含异常态） | ✅ |
| `core/guidance` | 巡航/末端/恢复/基座（含螺旋搜索）/紧急降落 | ✅ |
| `core/planner` | 固定容量航点队列 | ✅ |
| `core/safety` | 撞击检测（比力偏离 1g + 陀螺尖峰）、时间预算、围栏 | ✅ |
| `perception/camera` | 针孔模型、投影（仿真成像）、像素+尺寸→相对位置重建 | ✅ |
| `perception/target_tracker` | 目标相对位置滤波跟踪 | ✅ |
| `perception/home_detector` | 基座 marker 滤波跟踪 | ✅ |
| `perception/vision_frontend` | 视觉前端（特征/光流 → VO） | ⬜ 仅占位接口 |
| `flight/controller_interface` | GuidanceOutput → 加速度/偏航角速度指令 | ✅ |
| `flight/flight_state` | 飞控模式定义 | ✅ |
| `swarm/*` | 多机接口 | ✅ `other_count=0` |
| `platform/host` | 仿真主循环 | ✅ |
| `platform/stm32` | MCU 移植层 | ⬜ |
| `simulation` | 动力学（含姿态运动学）、IMU/VO/相机仿真、场景配置 | ✅ |

## 关键设计决策

- **INS + VO 融合**：IMU 做预测（高带宽、短时精确），VO 做互补校正
  （低带宽、有漂移）；VO 失效时纯 IMU 推算并按失效时长降级
  （TRACKING→DEGRADED→LOST），恢复后经 RECOVERING→RELOCALIZED 收敛。
- **姿态估计**：Mahony 互补滤波（加速度计倾斜修正 + 陀螺偏置学习），
  比力偏离 1g 超过 1.5 m/s² 时自动闭锁修正（机动/撞击期间不误修正）。
- **撞击检测**：比力模长偏离 1g（正常机动 <7，撞击 >100）+ 陀螺尖峰，
  连续帧确认；**进入 TARGET_TRACK（新一轮接近）时重新布防**，
  撞击锁存在 RECOVERY 流程中保持——估计器丢失恢复后仍能正确进入 IMPACT。
  仿真中撞击可重复注入（0.5 s 不应期），二次接近会再次触发接触。
- **仿真姿态模型**：推力矢量模型（机体 z 对齐比力方向）+ 姿态环带宽限制
  （5 rad/s），撞击扰动绕过带宽限制注入——正常飞行无虚假陀螺尖峰，
  撞击产生真实尖峰（~60 rad/s）。
- **相机检测**：前视相机找目标、下视相机找基座；像素 + 已知尺寸 →
  相对位置；检测距离由最小像素尺寸自然限制；视锥/图像边界决定可见性。
- **三层定位的闭环验证**：VO 常值漂移（~0.03 m/s 量级）→ 返航终点偏差
  → 螺旋搜索 → marker 捕获 → 重定位注入（1 s 冷却 + 最小校正门限防抖）
  → 视觉伺服停靠。
- **停靠判定（双判据）**：视觉对准判据（marker 可见 + 估计高度 ≤ dock_alt
  + 横向 ≤ 0.08 m）与**机械捕获判据**（已着陆 |v|≤0.08 m/s + 低空 +
  最后在捕获半径 0.15 m 内）。后者不依赖估计高度绝对精度——近地面
  marker 出画 + VO z 漂移时视觉判据可能永远不满足，机械捕获
  （基座停靠结构，Plan 允许自行设计）兜底；低高度 marker 短暂丢失时
  垂直盲降（≤0.35 m），不爬升重搜。

## 坐标系与约定

- 导航系：x/y 水平，z 向上；机体系由姿态四元数定义（机体 z 对齐推力方向）。
- 相机系：z 轴朝前（光轴），安装矩阵 `CAM_MOUNT_FORWARD` / `CAM_MOUNT_DOWN`。
- IMU 量测为机体系比力（含重力，悬停时约为 (0,0,+9.81)）。
- 全部模块：C99、`float32`、固定容量数组、静态内存、无 malloc/STL/异常/RTTI。

## 验证

- 单元测试：`test_math`（含四元数）、`test_impact_detector`、
  `test_mission_fsm`（正常链路 + 目标丢失/估计器丢失/超时返航）、
  `test_camera`（投影/重建往返、视锥剔除）、`test_ins_estimator`
  （零漂、姿态收敛、VO 校正收敛、健康机、撞击盲期）。
- 集成测试：`mission_closed_loop` + 蒙特卡洛 `mc_seed_1..16`、
  `mc_lost_11..20`（最坏情况：撞击直接 LOST）、`mc_truth_mode`
  （理想对照），共 33 项。
- 扩展手动扫掠（45 组：seed 1..30 正常 + 11..25 最坏情况）全部通过。
- 典型指标：任务耗时 ≤19 s（预算 30 s），停靠误差 ≤0.2 m，
  最大估计误差（VO 漂移，重定位前）≤0.8 m。

## 已知简化（后续迭代候选）

- VO 为仿真漂移模型，前端（光流/特征跟踪）未实现。
- 控制器为运动学级（理想低层飞控）；无电机/电调模型。
- 轨迹为 waypoint + 速度剖面；MINCO-lite 未实现。
- 目标检测假设已知目标尺寸；无分类器/检测网络。
- 撞击恢复假设低层姿态环总能稳住（未模拟翻倾坠地）。
