# RM_Drone_Nav

RoboMaster 微型无人机自主导航（Plan B：完全机载自主）。
总体设计见 `General_Plan.md`，模块划分与当前进度见 `docs/architecture.md`。

## 当前状态：第三/四阶段 —— 真实算法链路 + 翻倾恢复

host 仿真 + MCU 兼容核心（C99、静态内存、无动态分配、无外部依赖）。
完整任务闭环已在**真实算法链路**下跑通（40 项自动化测试）：

- **IMU**：机体系比力（含重力）+ 陀螺 + 上电随机偏置 + 噪声
- **INS 估计器**：陀螺姿态积分 + Mahony 倾斜修正 + 光流 VO 水平通道互补校正
  + **ToF 测距锚定 z 通道**（二阶互补，抑制加速度计 z 偏置积分）
- **光流 VO 前端**：id 关联 → 陀螺旋转补偿（标准光流方程）→ 平移最小二乘
  （尺度来自 ToF）→ 外点剔除重解 → 位置/偏航积分；漂移自然涌现
- **相机检测**：针孔投影 + 像素噪声；像素 + 已知尺寸 → 相对位置重建（PnP-lite）
- **轨迹**：航点间 minimum-jerk 五次多项式（MINCO-lite 低保底），前馈跟踪
- **动力学**：推力矢量模型——推力沿真实机体 z 轴，翻倾期间推力方向错误
  产生真实的高度/位置损失
- **撞击恢复**：比力偏离 1g + 陀螺尖峰检测；翻倾 >20° 时只做爬升保高度，
  改平后速度阻尼；倾角 + 估计器健康双条件退出
- **三层定位**：返航靠漂移 VO → 螺旋搜索 → marker 捕获 → 重定位 → 停靠
  （视觉对准 + 机械捕获双判据，低空盲降兜底）

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
./build/mission_sim.exe                       # 默认：INS + 光流 VO + ToF
./build/mission_sim.exe --seed 7              # 换随机种子（偏置/漂移/噪声不同）
./build/mission_sim.exe --lost-on-impact      # 最坏情况：撞击后估计器 LOST
./build/mission_sim.exe --hard-impact         # 剧烈撞击：>90° 翻倾恢复
./build/mission_sim.exe --sim-vo              # 对照：合成 VO（注入漂移模型）
./build/mission_sim.exe --truth-mode          # 对照：理想真值透传
```

退出码：0 = 任务完成；2 = 紧急降落；3 = 仿真超时。

运行全部测试（7 个单元测试 + 闭环集成 + 32 组蒙特卡洛回归，共 40 项）：

```sh
cd build && ctest --output-on-failure
```

## 开发顺序（Plan 第 15 节）

1. ✅ 真值传感器 + 状态机 + 制导 + 仿真动力学闭环
2. ✅ 真值状态 → IMU + 估计器融合（INS + Mahony + ToF + VO 互补）
3. ✅ 模拟目标 → 相机模型检测（像素 → 相对位置重建）
4. ✅ 视觉里程计前端（下视光流 + 陀螺补偿 + 最小二乘 + ToF 尺度）
5. ✅ MINCO-lite（min-jerk 五次多项式 + 前馈跟踪）
6. ✅ STM32 移植层骨架（platform/stm32：接口契约 + 固件装配参考）
7. ✅ 剧烈撞击/翻倾恢复策略
8. ⬜ 目标/基座检测的图像处理前端（blob 检测，像素接口已就位）
9. ⬜ 低层飞控（姿态/角速度环）实机实现或对接既有固件
