---

# RoboMaster 微型无人机自主导航 Plan B 当前情况总结

> 文档定位：本文件保留最初的方案论证、约束和阶段取舍。当前代码状态、测试数量、已实现的可选多机避碰接口及实机边界，以 `README.md` 和 `docs/architecture.md` 为准。

## 1. 项目目标

当前负责 RoboMaster 新规则中的**微型无人机/无人机群自主导航算法**。

无人机受到极严格的 SWaP 约束，即尺寸、重量、功耗和计算资源都非常有限。当前已知：

* 直径约 **140 mm** 时，最大重量约 **150 g**；
* 另一尺寸/重量约束点按当前信息为 **60 mm / 249 g**，但这两个数据的线性关系暂时不在算法阶段硬编码，后续以正式规则为准；
* 单次允许飞行时间最多约 **30 s**；
* 如需再次出动，需要返回出发点/基座进行**无线充电**；
* 基座可自行设计，因此可以将其设计成视觉定位、精确停靠甚至绝对位置校正的基础设施；
* 当前阶段**只开发算法，不绑定最终硬件方案**；
* 最坏情况下，最终高层导航和飞控可能都只能运行在 **STM32 级 MCU** 上。

任务的大体闭环是：

```text
基座
 ↓
起飞
 ↓
自主出航
 ↓
搜索/寻找目标
 ↓
视觉目标锁定
 ↓
目标接近
 ↓
接触目标
 ↓
接触后的姿态恢复
 ↓
定位恢复
 ↓
返航
 ↓
识别基座
 ↓
精确停靠
 ↓
无线充电
```

这里最特殊的两个问题是：

1. **接触目标时必须保证飞行器不彻底失控；**
2. **接触之后仍必须恢复定位并返回基座。**

---

## 2. 当前三个总体方案

目前保留三个技术方案。

### Plan A：依赖雷达兵种通信

如果比赛允许无人机从己方雷达系统获取可靠目标位置或自身辅助定位信息，那么这是最简单方案。

大致是：

```text
雷达/外部系统
    ↓
目标全局坐标
    ↓
无人机自身状态
    ↓
制导
    ↓
目标接近
```

Plan A 可以大量减少机载感知和搜索压力。

但当前不优先实现。

---

### Plan B：完全机载自主

这是目前**优先开发、也是最困难的方案**。

无人机不能依赖雷达提供目标信息，而是依靠：

```text
Camera
+
IMU
+
STM32
```

完成尽可能完整的：

```text
定位
目标识别/跟踪
制导
返航
停靠
异常恢复
```

最初的想法是：

> “把高飞组论文算法阉割以后部署 STM32。”

现在需要明确否定这种理解。

高飞组原论文的系统实际上把计算分成两层：

```text
STM32 H7 + BMI088
        ↓
低层飞控

Xavier NX
        ↓
VIO
Mapping
Planning
High-level control
其他任务算法
```

论文明确指出 STM32 H7 主要承担低层姿态控制和 IMU 数据处理，而定位、规划和高层控制运行在 Xavier NX。

原系统还使用了 RealSense D430、UWB、VINS 等设备和算法。

因此 Plan B **不是直接移植论文代码**，而是：

> 从论文系统中抽取必要思想，在 STM32 级资源限制下重新设计一个任务专用的最小自主导航系统。

---

### Plan C：极简保底

如果 Plan B 最终因硬件/时间原因失败：

```text
IMU
+
惯导/短时间航迹推算
+
预设轨迹
+
开环/半开环控制
```

飞出去执行任务，然后尽可能返航。

Plan C 应当尽量由 Plan B 的模块自然降级得到，而不是单独再写一套系统。

---

# 3. 非常重要：目前不假设真正的“集群同时飞行”

实际战术很可能是：

```text
Drone 1 出动
→ 返回

Drone 2 出动
→ 返回

Drone 3 出动
→ 返回
```

也就是：

> **多机系统，但单机串行执行任务。**

因此第一版 Plan B **不实现完整 swarm planning**。

这意味着高飞组论文中的大量内容可以暂时删除：

* 编队保持；
* 多机 reciprocal avoidance；
* 多机轨迹实时广播；
* 多机 UWB 相互纠漂；
* 多机拓扑；
* formation optimization；
* 多机死锁处理。

高飞组 EGO-Swarm 的这些设计主要针对多个无人机同时在未知环境中自主运行，其核心包括轨迹共享和多机避碰。

---

# 4. 但是架构必须保留未来并行飞行的扩展空间

虽然第一版按**单机串行**开发，但不能把代码写死成永远只有一个无人机。

建议从一开始保留：

```text
agent_id

timestamp

self_state

other_agent_state[]

other_agent_trajectory[]

communication interface

collision object abstraction
```

第一版：

```text
other_agent_count = 0
```

未来并行飞行时：

```text
other_agent_count > 0
```

即可增加：

```text
多机通信
→ 接收其他无人机轨迹
→ 将其他无人机视为动态障碍
→ trajectory conflict detection
→ reciprocal avoidance
```

高飞组自己就是用“共享未来轨迹”的方式进行去中心化多机避碰，而通信负载实际上并不要求传整张地图。其补充材料中，10 架无人机竹林实验的一条轨迹约 170 B，平均通信量只有约 2 kB/s、峰值约 8 kB/s。

因此未来并行飞行可以扩展，但**现在不应该为了这个能力支付主要计算成本**。

---

# 5. Plan B 第一版的核心架构

建议 K3 按这个方向设计：

```text
                  ┌──────────── Camera ────────────┐
                  │                                │
                  ↓                                ↓
          Lightweight Odometry             Target Detection
                  │                                │
IMU ───────→ State Estimator                      │
                  │                                │
                  └────────────┬───────────────────┘
                               ↓
                         Mission FSM
                               │
               ┌───────────────┼───────────────┐
               ↓               ↓               ↓
            Cruise          Terminal        Return
           Guidance         Guidance        Guidance
               │               │               │
               └───────────────┴───────────────┘
                               ↓
                     Position/Velocity Target
                               ↓
                         Flight Controller
                               ↓
                           ESC/Motors
```

以后如果加入多机：

```text
Other Agents' Trajectories
          ↓
Dynamic Object Interface
          ↓
Planner
```

这样不会破坏单机架构。

---

# 6. 定位系统不要假设“VIO 从起飞到回家永远连续”

这是整个 Plan B 的重要设计原则。

普通阶段可以使用：

```text
Camera + IMU
      ↓
轻量视觉惯性里程计
      ↓
Local Odometry
```

但是接触目标时可能发生：

```text
巨大瞬时加速度
+
大角速度
+
Motion Blur
+
视觉特征骤减
+
IMU 饱和
+
振动
```

论文补充材料甚至专门提到，发生 crash 后可能出现初始状态估计异常，并建议重新校准 camera-IMU 外参。

因此整个任务不能写成：

```text
VIO启动
↓
一直积分30秒
↓
直接靠同一个坐标系回家
```

而应该允许 estimator：

```text
TRACKING
DEGRADED
LOST
RECOVERING
RELOCALIZED
```

---

# 7. 建议采用“三层定位思想”

## 第一层：航行定位

负责出去和大范围返航：

```text
Camera + IMU
→ lightweight VIO / visual odometry
```

目标不是厘米级全程绝对定位。

目标只是：

> 短时间内提供连续、足够好的局部运动状态。

30 秒任务窗口其实对此非常有利。

---

## 第二层：目标相对定位

一旦发现目标：

```text
Camera
 ↓
Target Detector
 ↓
目标中心 / 相对姿态 / 相对尺度
 ↓
Terminal Guidance
```

此时尽量减少对全局坐标的依赖。

可以转成：

```text
relative x
relative y
relative z
relative yaw
```

或者最简单：

```text
image error ex
image error ey
target scale
```

然后视觉伺服接近。

---

## 第三层：基座绝对定位

返航阶段：

```text
粗略 odometry
 ↓
回到 Home Region
 ↓
Camera 搜索基地标志
 ↓
Base Marker Detection
 ↓
PnP / Visual Servoing
 ↓
精确降落
```

因为**基座自己设计**，所以完全应该利用这一点。

可以主动给它设计：

* 高对比视觉图案；
* AprilTag/自定义 fiducial；
* 多尺度标志；
* LED；
* 方向编码；
* 近距离精确停靠结构。

这样就算出去以后 VIO 有几十厘米甚至更大的漂移：

> 只要能回到基地附近，就可以重新建立绝对参考。

---

# 8. 接触目标必须独立成一个状态模块

建议定义：

```text
IMPACT_RECOVERY
```

不要把它当普通导航中的一个瞬时扰动。

状态流程：

```text
TERMINAL
   ↓
Impact Detect
   ↓
IMPACT
   ↓
短暂视觉冻结/拒绝异常视觉
   ↓
Attitude Stabilization
   ↓
Velocity Damping
   ↓
Estimator Health Check
   ↓
┌──────────────┬──────────────┐
│ estimator OK │ estimator bad│
↓              ↓
BREAKAWAY     RELOCALIZE
│              │
└───────┬──────┘
        ↓
    RETURN_HOME
```

Impact detection 可以后续使用：

```text
accelerometer norm
gyro spike
motor response
attitude error
```

组合判定。

姿态恢复主要应该由低层飞控承担。

导航层不要自己直接控制电机。

---

# 9. 飞控和导航必须分层

建议保持：

```text
Mission
 ↓
Guidance / Planning
 ↓
Desired position / velocity / yaw
 ↓
Position Controller
 ↓
Attitude Controller
 ↓
Rate Controller
 ↓
Motor Mixer
 ↓
ESC
```

其中 STM32 上最高优先级必须是：

```text
IMU
Rate Controller
Attitude Controller
Motor Output
```

即使：

```text
视觉挂了
规划挂了
定位挂了
```

也不能影响基本姿态稳定。

高飞组原系统同样将低层 FCU 和高层导航分开。

---

# 10. 第一版不要实现完整 MINCO

高飞组论文最核心的亮点之一是 MINCO / spatial-temporal optimization。

它同时优化：

```text
空间轨迹形状
+
每段时间分配
```

论文认为这可以明显改善复杂环境以及多无人机情况下的轨迹质量。

但它现在**不是我们第一优先级**。

第一版可以先：

```text
Waypoint
+
Polynomial / simple spline
+
velocity profile
```

甚至：

```text
position target
→ velocity command
```

都可以。

只有当：

```text
VIO
目标检测
Target Guidance
Impact Recovery
Return Home
Docking
```

全部已经跑通，再考虑：

```text
MINCO-lite
```

或者其他固定维度轨迹优化。

---

# 11. 第一版甚至不一定需要在线建图

比赛场地通常是已知环境。

所以先设计成：

```text
Known Arena Geometry
       ↓
predefined corridor / waypoint graph
```

局部只做：

```text
camera / ToF
 ↓
obstacle warning
 ↓
local avoidance / emergency stop
```

而不是复现论文：

```text
Depth
→ probabilistic occupancy mapping
→ online planning
```

高飞组必须在线建图，是因为其目标就是在**先验未知的杂乱野外环境**中自主飞行。

你的任务环境和研究目标不同。

---

# 12. STM32 软件实现约束

K3 开发时默认：

### 允许

```text
C99
C++11/14 subset
CMSIS-DSP
固定大小矩阵
float32
FreeRTOS（未来可选）
静态内存
固定容量 ring buffer
```

### 禁止依赖

```text
ROS / ROS2
OpenCV runtime
Ceres
g2o
Eigen 动态矩阵
Python runtime
STL 大量动态容器
malloc/free 高频使用
exceptions
RTTI
GPU
操作系统文件系统
```

### 核心原则

所有算法应该：

```text
输入结构体
    ↓
纯算法函数
    ↓
输出结构体
```

例如：

```c
nav_state_update(...)

impact_detector_update(...)

mission_fsm_update(...)

target_guidance_update(...)

home_guidance_update(...)
```

这样以后：

```text
PC 仿真
STM32
HIL
真实无人机
```

都能复用。

---

# 13. 推荐的软件模块

建议第一版仓库直接拆：

```text
core/
    math/
    filters/
    state_estimator/
    mission/
    guidance/
    planner/
    safety/

perception/
    vision_frontend/
    target_tracker/
    home_detector/

flight/
    controller_interface/
    flight_state/

swarm/
    agent_state/
    trajectory_message/
    collision_interface/

platform/
    host/
    stm32/

tests/
simulation/
docs/
```

其中：

```text
swarm/
```

第一版可以几乎为空，但接口必须保留。

---

# 14. Mission FSM 第一版建议

```text
BOOT
↓
SELF_CHECK
↓
DOCKED
↓
TAKEOFF
↓
OUTBOUND
↓
SEARCH
↓
TARGET_TRACK
↓
TERMINAL
↓
IMPACT
↓
RECOVERY
↓
BREAKAWAY
↓
RETURN_HOME
↓
HOME_SEARCH
↓
DOCKING
↓
DOCKED
```

异常状态：

```text
ESTIMATOR_LOST
TARGET_LOST
LOW_BATTERY
NAV_FAILURE
EMERGENCY_STABILIZE
EMERGENCY_LAND
```

---

# 15. Plan B 第一阶段真正要 K3 做的东西

第一阶段**不要做完整视觉算法，也不要做硬件驱动**。

先建立算法骨架：

```text
真值/模拟传感器
        ↓
State Estimator abstraction
        ↓
Mission FSM
        ↓
Guidance
        ↓
Simple trajectory
        ↓
Simulated flight dynamics
```

并验证完整任务：

```text
起飞
→ 飞向搜索区
→ 获取模拟目标
→ 接近
→ 模拟 impact
→ recovery
→ return
→ home acquisition
→ docking
```

先把这个闭环完整跑通。

随后再逐渐把：

```text
真值状态
```

替换成：

```text
IMU
→ estimator
```

再替换：

```text
模拟目标
```

为：

```text
camera target detector
```

最后再处理：

```text
视觉里程计
```

这是风险最低的开发顺序。

---

# 16. 当前 Plan B 的一句话定义

现在可以正式把 Plan B 定义成：

> **在 STM32 级计算资源约束下，为 RoboMaster 微型无人机设计一套任务专用的机载自主导航系统，在不依赖雷达外部目标信息的情况下，实现短时自主出航、目标视觉接近、接触事件处理、姿态与定位恢复、返航和基座视觉精确停靠；第一阶段按单机串行任务实现，但从数据结构、通信接口、动态障碍表示和轨迹接口上保留未来多机并行飞行与轨迹共享避碰能力。**

而整个开发最重要的原则可以再压缩成一句：

> **不是把高飞组系统缩小，而是重新设计一套 STM32 能承担的最小自主闭环。**

这个版本已经足够让 K3 开始做第一轮架构和代码了。建议让它**先实现 host 仿真 + MCU-compatible core，不要一上来碰 STM32 HAL，也不要一上来写 VIO/MINCO**；先把完整状态机、数据结构、制导接口、impact recovery 和未来 swarm 扩展接口建立起来。
