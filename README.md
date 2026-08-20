# RM_Drone_Nav

面向 RoboMaster 微型无人机的 Plan B 自主导航算法原型。项目以 STM32 级资源约束为设计边界，在主机仿真中打通：起飞、出航、目标搜索与接近、受控接触、撞击后恢复、返航、基座重定位和停靠。

> 当前结论：这是可编译、可回归的 C99 算法原型和 STM32 移植骨架，不是已经完成实机飞行验证的固件。图像检测器、板级驱动、低层姿态控制和硬件参数标定仍需在目标平台完成。

## 已实现能力

| 子系统 | 当前实现 |
|---|---|
| 状态估计 | IMU 姿态/惯性预测、光流 VO 水平修正、ToF 高度融合、视觉绝对修正门限，以及 INIT/DEGRADED/LOST/RECOVERING/RELOCALIZED 等健康状态 |
| 目标与基座跟踪 | 固定内存 alpha-beta 相对状态跟踪、置信度与观测年龄、短时预测、LOST 后重新建轨；基座相对位置和偏航滤波 |
| 任务管理 | 19 个顶层状态，包含目标确认与丢失、受控接触、分阶段恢复、强制返航、基座搜索、精细归航、停靠和紧急降落 |
| 末端制导 | PURSUIT/CLOSING/FINAL_ALIGN/CONTACT_APPROACH/REACQUIRE 分层，速度/加速度限制和移动目标速度前馈 |
| 撞击处理 | 加速度、角速度、速度跳变、姿态跳变融合；布防、去抖、不应期；视觉保持、姿态恢复、速度阻尼、估计器检查和脱离 |
| 轨迹规划 | 固定容量五次多项式轨迹；速度、加速度、时长、工作空间和碰撞后检查；有界时间拉伸；固定内存局部绕行和简单制导兜底 |
| 安全管理 | 软/硬任务截止时间、围栏、估计器质量、轨迹有效性、控制饱和和碰撞风险分级；短时风险去抖 |
| 动态障碍 | 最多 8 个局部/动态障碍，最近接预测、横向避让和紧急爬升输出 |
| 多机扩展 | 最多 4 个他机状态、带时间戳未来轨迹消息、超时处理、未来冲突检测，以及按 `agent_id` 确定性让行；默认可关闭 |
| STM32 对接 | 与主机同序的 `NavApp` 调用链和板级接口契约；无堆分配、固定容量数组 |

算法选择参考了 EGO-Planner、EGO-Swarm、分布式群体轨迹优化、bearing 相对定位、《Swarm of micro flying robots in the wild》及 GCOPTER。项目只提取固定维度轨迹、约束后检查、轨迹时标缩放、带时效的未来状态共享和相对观测恢复等思想；未把 ESDF、ROS、完整 VIO、L-BFGS 或一般非线性优化器直接搬到 MCU。

## 构建与验证

依赖 CMake 3.16+ 和 C99 编译器。Windows MinGW 示例：

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

默认仿真：

```powershell
.\build\mission_sim.exe
.\build\mission_sim.exe --list-scenarios
.\build\mission_sim.exe --scenario target-loss
.\build\mission_sim.exe --scenario local-obstacle
.\build\mission_sim.exe --scenario two-agent-conflict
.\build\mission_sim.exe --lost-on-impact --seed 17
.\build\mission_sim.exe --hard-impact --seed 23
```

退出码：`0` 为任务完成并停靠；`2` 为紧急降落；`3` 为仿真超时；`4` 表示场景虽然到达终点，但声明的压力分支没有真正触发。

当前 CTest 共 53 项：

- 11 个模块级测试；
- 1 个默认闭环测试；
- 9 个具名压力场景；
- 32 个随机种子、撞击后 LOST、剧烈撞击及真值对照回归。

具名场景包括 `moving-target`、`target-loss`、`impact-degraded`、`impact-lost`、`home-initial-hidden`、`home-loss`、`local-obstacle`、`forced-return` 和 `two-agent-conflict`。场景程序会检查相应异常分支确实被执行，而不只检查最终出现 `MISSION_SUCCESS`。

## 目录边界

```text
core/            MCU 兼容的数学、估计、FSM、制导、规划和安全算法
perception/      相机模型、光流前端、目标/基座跟踪
flight/          导航到低层飞控的控制接口
swarm/           可关闭的多机状态、轨迹消息和冲突处理
platform/host/   主机闭环仿真入口
platform/stm32/  STM32 板级接口与 NavApp 装配骨架
simulation/      仅主机使用的动力学、传感器和场景模型
tests/           单元、集成和随机回归测试
docs/            架构和设计说明
```

CMake 将 `nav_core`、`nav_sim` 和 `nav_stm32_port` 分开构建。固件只需要 `nav_core` 与 `nav_stm32_port`，不应链接 `simulation/`。

## STM32 对接路径

1. 依据 `simulation/scenario.c` 将经过实机标定的参数填入静态 `NavAppConfig`；不要原样照搬仿真参数。
2. 实现 `platform/stm32/nav_platform.h` 中的时钟、IMU、ToF、光流、双相机、局部障碍、任务命令、停靠状态、多机通信和 FCU 输出接口。
3. 以 100–200 Hz 调用 `nav_app_step`。IMU 采样、姿态/角速度环、混控和电机输出必须运行在更高优先级。
4. 在解锁前完成坐标系、单位、时间戳回绕、传感器失效、围栏、紧急指令和停靠触点的台架测试。

低层姿态稳定不能依赖导航任务。`CtrlOutput` 只提供期望加速度与偏航角速度，不能直接驱动电机。

## 已知边界

- 未实现真实图像中的特征提取、目标分类或 marker 解码；当前接口从像素观测开始。
- 无稠密地图、一般三维走廊搜索或完整优化规划器；局部绕行只适合少量已知障碍。
- 多机模式已能验证通信超时、未来冲突和确定性让行，但没有网络协议、时钟同步和实机通信测试。
- 没有真实电机、电池、气动、接触结构或无线充电模型。
- 尚未在目标 STM32、传感器和机体上测量 RAM、最坏执行时间与控制稳定裕量。
- 30 秒预算是仿真安全约束；实机必须根据电池和比赛规则留出更大的返航裕量。

进一步说明见 [架构文档](docs/architecture.md)、[总体方案](General_Plan.md) 和 [STM32 移植说明](platform/stm32/README.md)。
