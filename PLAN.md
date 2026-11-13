# g1d_ws 开发计划 / TODO / 进度

> 设计依据：`../design/G1D_LIO_MPPI_Nav_Plan.md`（G1-D 头部 Mid-360S 激光里程计 + 避障导航项目方案）。
> 本文件在开发过程中持续更新：计划（第 1 节）、TODO（第 3 节）、进度日志（第 5 节）。

## 1. 目标与范围

在 `g1d_ws/` 中实现方案文档描述的完整代码：

| 进程 | 语言 | 内容 |
|---|---|---|
| `g1d_lio` | C++17 | Livox-SDK2 直连 Mid-360S → FAST-LIO2（去 ROS）→ DDS 发布 LIO 位姿 / 配准点云 |
| `g1d_nav` | C++17 | FK + 状态估计 + 代价地图（Nav2 costmap_2d 核心）+ NavFn + MPPI（Nav2 Eigen 版）+ 导航状态机 |
| `g1d_base_guard` | Python | `rt/cmd_vel_no_limit` 唯一写入者：限幅 / 看门狗 / 反射层 / 事件急停 |
| `navgpt_bridge` / `NavClient` | Python | NavGPT 目标下发与状态反馈 |
| tools | Python | 接口探测、`cmd_vel_no_limit` 行为实测、录制、外参标定辅助 |

机器人上不依赖 ROS；进程间通信只用宇树 SDK2（CycloneDDS）。

## 2. 开发与验证策略（本机 = Windows + MinGW g++13 + Python3.12）

本机只做开发，运行环境是 Jetson Orin NX（Ubuntu 20.04 / aarch64），代码之后手动移植过去。

| 层 | 本机怎么验证 | 目标机 |
|---|---|---|
| C++ 算法核心（`g1d_core` 静态库：common / lio / nav） | MinGW 编译 + ctest 单元测试 + 合成数据测试 | 同代码 |
| FAST-LIO2 | 合成 Mid-360 点云 + IMU（已知真值轨迹）→ 比较估计误差 | 实机 |
| 代价地图 / NavFn / MPPI / 状态机 | 进程内闭环仿真（差速底盘 + 场景 + 合成雷达）：绕障到达、无碰撞 | 实机 |
| C++ DDS 适配层（unitree_sdk2） | 本机无法编译（SDK 只提供 Linux 库）；`-DG1D_WITH_UNITREE=ON` 只在 Linux 启用 | 编译 + 实测 |
| Livox-SDK2 接入 | 帧聚合 / 逐点时间逻辑做成纯函数，用合成 packet 单测；SDK 调用只在 Linux 启用 | 编译 + 实测 |
| Python（guard / client / mock / tools） | pytest + **真实 DDS 回环**（本机 cyclonedds 11.1 + unitree_sdk2py 可用） | 同代码（cyclonedds 0.10.2） |

## 3. TODO（随进度勾选）

### M0 仓库与依赖
- [ ] git 初始化、目录骨架、`.gitignore`、本文件
- [ ] submodule：eigen 3.4.0、yaml-cpp、Livox-SDK2、unitree_sdk2、unitree_sdk2_python
- [ ] vendor（先原样提交、再改）：FAST_LIO 核心 + ikd-Tree；Nav2 子集（costmap_2d 核心、voxel_grid、navfn、mppi_controller、nav2_util/nav2_core 头文件）
- [ ] 顶层 CMake（MinGW / Linux 两套），ctest

### M1 common
- [ ] `ros_shim/`：让 Nav2 代码不改或少改即可编译的最小 ROS 头文件（rclcpp 日志/参数/时间、geometry_msgs、nav_msgs、tf2、angles、pluginlib 静态注册、nav2::LifecycleNode）
- [ ] `pcl_lite/`：FAST-LIO 用到的 PCL 子集（点类型、PointCloud、VoxelGrid）
- [ ] `g1d/common`：SE2/SE3、YAML 配置、时间、PoseBuffer、日志

### M2 LIO
- [ ] FAST-LIO2 去 ROS：`FastLio` 类（push_imu / push_frame / process）
- [ ] Livox 帧聚合 + 逐点时间（纯函数）+ 自身点过滤 + 健康监测
- [ ] 合成 Mid-360 仿真器 + 精度测试

### M3 NAV
- [ ] BodyKinematics（URDF 链）+ StateEstimator（LIO + 轮速外推 + 位姿历史）
- [ ] Costmap：Costmap2D / VoxelGrid / footprint / 膨胀；VoxelDecayLayer、ScanLayer、LayeredCostmap（local + global）
- [ ] NavFn 规划 + 目标合法性检查
- [ ] MPPI 移植（Eigen 版）+ CostmapView 垫片 + parity 冒烟测试
- [ ] NavManager 状态机 + free_sectors + goal/status JSON
- [ ] 闭环仿真测试（绕障、目标在障碍内、动态行人、到达精度）

### M4 DDS 进程（Linux）
- [ ] `g1d_lio` / `g1d_nav` 可执行程序（unitree_sdk2）+ `LaserScan_` IDL
- [ ] 本机 `-fsyntax-only` 尽量检查

### M5 Python
- [ ] `g1d_nav_py`：idl（LaserScan_）、base_guard、NavClient、mock_robot、tools
- [ ] pytest：guard 逻辑单测 + DDS 回环端到端（mock 底盘 ↔ guard ↔ 模拟 nav）

### M6 部署与文档
- [ ] `config/`、`deploy/`（cyclonedds.xml、systemd、run 脚本）、`scripts/build_*.sh/.ps1`
- [ ] README（构建、移植到 Jetson、运行、验收步骤）

## 4. 关键设计决策

| 决策 | 原因 |
|---|---|
| Nav2 代码用"原样 vendor + ros_shim 垫片 + 少量补丁" | 保持算法与上游一致，补丁集中、可追溯；便于后续合入上游更新 |
| FAST-LIO 用 `pcl_lite` 代替 PCL | FAST-LIO 只用到 PCL 的点类型 / PointCloud / VoxelGrid；去掉 PCL 后 Windows 与 Jetson 都无需安装 PCL |
| 未修改的依赖用 git submodule，修改过的上游代码 vendor 进仓库 | submodule 保持可追溯；vendor 代码的每处修改都在本仓库 git 历史里 |
| C++ 核心与 DDS 适配分离 | 本机无法链接 unitree_sdk2，核心逻辑仍能完整测试 |

## 5. 进度日志

| 时间 | 进度 |
|---|---|
| 2026-11-13 | 环境确认：MinGW g++13.2、Python3.12、pip 安装 cmake/ninja/pytest/cyclonedds 11.1；unitree_sdk2py 在本机 DDS 回环测试通过。上游克隆到 `_upstream/`：FAST_LIO@7cc4175、FAST-LIO-NON-ROS、livox_ros_driver2、navigation2@4c01575（sparse） |
