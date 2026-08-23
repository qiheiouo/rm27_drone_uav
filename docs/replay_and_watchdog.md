# 日志回放与导航 watchdog

## 1. 运行时健康检查

`NavRuntime` 在每次更新前检查时间基准和输入样本：

- IMU 是必需输入；缺失、过期、重复、乱序或非有限值会拒绝本次更新并输出零导航指令；
- 光流、外部里程计、目标像素和基座像素是可选输入；异常样本会被丢弃，其他模块继续按降级路径运行；
- 所有毫秒时间戳使用无符号差值，允许 32 位计数器正常回绕；
- 小幅未来时间戳由 `future_tolerance_ms` 容忍，超过门限视为时间基准异常；
- 主循环间隔超过 `max_cycle_gap_ms` 时记录 overrun，连续达到 `watchdog_trip_after_overruns` 后锁存 watchdog，并通过任务状态机进入安全分支。

默认门限位于 `NavRuntimeConfig.health`。实机必须根据实际采样率、总线延迟和 RTOS 调度测量重新标定，不能直接把仿真值作为飞行参数。

`NavRuntimeOutput.health` 给出本周期的过期、重复、乱序、非有限值和无效输入掩码，以及周期间隔、连续超时次数和 watchdog 锁存状态。

## 2. 固定内存事件记录

`NavEventLog` 是 64 条记录的环形缓冲区。容量耗尽时覆盖最早记录并增加 `dropped`，不会分配堆内存。记录包括：

- 输入过期、重复、乱序、无效或非有限；
- watchdog 周期超时和锁存；
- 任务状态转换；
- 撞击、重定位、障碍和蜂群冲突；
- 轨迹绕行和结构性轨迹拒绝。

STM32 适配层会把新增事件名称转交给板级 `nav_log`。真实产品应让 `nav_log` 写入低优先级遥测或无阻塞队列，不能在导航任务中执行阻塞式串口输出。

## 3. CSV 回放工具

构建后运行：

```powershell
.\build\nav_replay.exe tests\data\replay_stationary.csv
.\build\nav_replay.exe --verify tests\data\replay_stationary.csv
```

普通模式逐帧输出导航状态和诊断事件。`--verify` 在两个全新运行时中回放同一文件，并比较量化后的状态摘要、样本数和事件记录，用于检查确定性。

CSV schema v1 每行固定 31 列：

```text
timestamp_ms,
ax,ay,az,gx,gy,gz,tof,
odom_valid,px,py,pz,vx,vy,vz,yaw,yaw_rate,
target_visible,target_u,target_v,target_size,
home_visible,home_u,home_v,home_size,
start,return,emergency,dock,charging,wireless_ready
```

约定：

- 坐标、单位和四元数方向与项目其余部分一致，全部采用 SI 单位；
- CSV 的里程计通道直接进入 `OdomSample`，用于回放板级 VO/VIO 输出，不回放可变长度光流特征集合；
- `target_visible` 或 `home_visible` 为零时，对应像素字段被忽略；
- 行以 `#` 开头时作为注释跳过；
- 文件中时间戳必须单调，停顿会由 watchdog 记录，重复或倒序的必需输入会使回放失败。

该格式用于确定性算法回归，不代替高带宽原始图像或总线抓包。后续接入实物时，应同时保存原始数据与这一份归一化导航输入日志。

## 4. 结构化输出遥测

CSV 回放记录“送入运行时的输入”，二进制遥测记录“运行时产生的状态与控制结果”。后者采用固定 152 字节、显式小端序、schema 版本和 CRC32，可由主机仿真直接录制，也可由 STM32 适配层投递到非阻塞板级队列。格式定义、离线检查和带宽约束见[结构化遥测](telemetry.md)。
