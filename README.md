# RM_Drone_Nav

RoboMaster 微型无人机自主导航（Plan B：完全机载自主）。
总体设计见 `General_Plan.md`，模块划分与当前进度见 `docs/architecture.md`。

## 当前状态：第二阶段 —— 真实传感链路闭环

host 仿真 + MCU 兼容核心（C99、静态内存、无动态分配、无外部依赖）。
完整任务闭环已在**真实传感模型**下跑通：

- IMU：机体系比力（含重力）+ 陀螺 + 固定偏置 + 噪声
- INS 估计器：陀螺姿态积分 + Mahony 倾斜修正 + VO 互补校正，
  健康状态机（TRACKING/DEGRADED/LOST/RECOVERING/RELOCALIZED）由 VO 有效性驱动
- 相机：针孔投影 + 像素噪声；目标/基座由像素 + 已知尺寸重建相对位置（PnP-lite）
- VO：带常值漂移与撞击 blackout；返航末端靠基座 marker 重定位消除漂移
- 撞击：比力偏离 1g + 陀螺尖峰组合检测；姿态环带宽受限，撞击绕带宽注入

```
起飞 → 出航 → 搜索 → 目标跟踪 → 末端接近 → 接触目标（真实撞击脉冲）
→ 恢复（速度阻尼 + 估计器健康检查）→ 撤离 → 返航（VO 漂移）
→ 螺旋搜索 → marker 捕获 → 重定位 → 视觉伺服停靠
```

## 构建与运行

依赖：CMake + 任一 C99 编译器（Windows 下已验证 clang + Ninja）。

```sh
cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=clang
cmake --build build
```

运行闭环仿真：

```sh
./build/mission_sim.exe                       # 默认：INS+VO 融合
./build/mission_sim.exe --seed 7              # 换随机种子（偏置/漂移/噪声不同）
./build/mission_sim.exe --lost-on-impact      # 最坏情况：撞击后估计器 LOST
./build/mission_sim.exe --truth-mode          # 对照组：理想真值透传
```

退出码：0 = 任务完成；2 = 紧急降落；3 = 仿真超时。

运行全部测试（5 个单元测试 + 闭环集成 + 27 组蒙特卡洛回归，共 33 项）：

```sh
cd build && ctest --output-on-failure
```

## 开发顺序（Plan 第 15 节）

1. ✅ 真值传感器 + 状态机 + 制导 + 仿真动力学闭环
2. ✅ 真值状态 → IMU + 估计器融合（INS + Mahony + VO 互补）
3. ✅ 模拟目标 → 相机模型检测（像素 → 相对位置重建）
4. ⬜ 视觉里程计前端（光流/特征 → VO，替换仿真 VO）
5. ⬜ MINCO-lite / 多项式轨迹优化（当前 waypoint + 速度剖面已够用）
6. ⬜ STM32 平台移植（platform/stm32）
