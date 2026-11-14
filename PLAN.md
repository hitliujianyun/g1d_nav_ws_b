# g1d_ws 开发计划 / TODO / 进度

> 设计依据：`../design/G1D_LIO_MPPI_Nav_Plan.md`（G1-D 头部 Mid-360S 激光里程计 + 避障导航项目方案）。
> 本文件在开发过程中持续更新：计划（第 1 节）、TODO（第 3 节）、进度日志（第 5 节）、上机待办（第 6 节）。
> 使用说明见 [`README.md`](README.md)，标定见 [`docs/CALIBRATION.md`](docs/CALIBRATION.md)。

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
| C++ 算法核心（common / lio / nav） | MinGW 编译 + ctest 单元测试 + 合成数据测试 | 同代码 |
| FAST-LIO2 | 合成 Mid-360 点云 + IMU（已知真值轨迹）→ 比较估计误差 | 实机 |
| 代价地图 / NavFn / MPPI / 状态机 | 进程内闭环仿真（差速底盘 + 场景 + 合成雷达）：绕障到达、无碰撞 | 实机 |
| C++ DDS 适配层（unitree_sdk2） | 本机无法链接（SDK 只提供 Linux 库）；对 SDK 头文件做 `-fsyntax-only` 检查 | 编译 + 实测 |
| Livox-SDK2 接入 | 帧聚合 / 逐点时间做成纯函数并单测；SDK 调用同上做语法检查 | 编译 + 实测 |
| Python（guard / client / mock / tools） | pytest + **真实 DDS 回环**（本机 cyclonedds 11.1 + unitree_sdk2py）+ 多进程冒烟 | 同代码（cyclonedds 0.10.2） |
| 构建系统 | CMake 3.16 兼容（老版本 whole-archive 分支本机强制测试）；打包检查 LF / 符号链接 / 可执行位 | `scripts/build_robot.sh` |

## 3. TODO（随进度勾选）

### M0 仓库与依赖
- [x] git 初始化、目录骨架、`.gitignore`、`.gitattributes`（LF）、本文件
- [x] submodule：eigen 3.4.0、yaml-cpp 0.8.0、Livox-SDK2、unitree_sdk2、unitree_sdk2_python、boost_preprocessor
- [x] vendor（先原样提交、再改）：FAST_LIO@7cc4175 核心 + ikd-Tree + IKFoM；Nav2@4c01575 子集
- [x] 顶层 CMake（MinGW / Linux），ctest

### M1 common
- [x] `ros_shim/`：rclcpp 日志/参数/时间、消息结构体、tf2、angles、pluginlib 静态注册、nav2::LifecycleNode、Costmap2DROS 门面
- [x] `pcl_lite/`：点类型、PointCloud、VoxelGrid
- [x] `g1d/common`：SE2/SE3、YAML 配置、时间（ClockMapper）、PoseBuffer

### M2 LIO
- [x] FAST-LIO2 去 ROS / PCL / Boost：`fast_lio_api`（init / push_imu / push_scan / process / reset）
- [x] Livox 帧聚合 + 逐点时间 + 自身点过滤 + 健康监测（`LioEstimator`）
- [x] 合成 Mid-360 仿真器 + 精度测试（最大误差 0.05 m / 0.09°）

### M3 NAV
- [x] BodyKinematics（URDF 链）+ StateEstimator（LIO + 轮速外推 + 位姿历史 + 一致性检查）
- [x] Costmap：Costmap2D / footprint / 膨胀（Nav2 原码）；VoxelLayer（列位掩码 + 3D 射线清除，保留悬空物）、ScanLayer、滚动 LayeredCostmap（local + global）
- [x] NavFn 规划 + 目标合法性检查（障碍内目标外移、超距截断）
- [x] MPPI 移植（Eigen 版，11 个 critic）+ Costmap2DROS 垫片
- [x] NavCore 状态机（navigate / rotate / simple / stop、`base_at_stamp` 延迟补偿、末段接近、恢复 wait / clear / rotate）+ free_sectors + goal/status JSON
- [x] 闭环仿真 8 个场景全部无碰撞（整机 0–1.75 m 高度检测）

### M4 DDS 进程（Linux）
- [x] `g1d_lio` / `g1d_nav` 可执行程序（unitree_sdk2）；底盘 `LaserScan_` 由 guard 转成 `PointCloud2_`（`rt/nav/chassis_scan`），C++ 无需 idlc
- [x] 本机 `-fsyntax-only` 检查（发现并修复 `g1d::dds` 与 CycloneDDS `::dds` 命名冲突）
- [x] 调试输出：`rt/nav/local_costmap`、`rt/nav/livox_cloud`（标定用，默认关）
- [x] 工具：`g1d_sim_demo`（离线闭环 + `tools/plot_sim_trace.py` 画图）、`g1d_fk`

### M5 Python
- [x] `g1d_nav_py`：`slamware_idl`（LaserScan_）、`extra_idl`（Imu_）、`guard_core` / `guard_node`、`nav_client`、`navgpt_bridge`、`mock_robot`、`calib`、`chassis_http`、`dds_discovery`
- [x] tools：`check_interfaces`、`cmd_vel_probe`、`record`、`nav_cli`、`calib_floor`
- [x] pytest 28 项：guard 逻辑 14 项、消息 / mock / 事件、标定算法、NavGPT 桥、DDS 回环端到端（mock 底盘 ↔ guard ↔ 模拟 nav；NavClient ↔ 模拟 nav）
- [x] 多进程冒烟：mock 机器人 + guard 各自独立进程，`check_interfaces` / `record` 跨进程收数据

### M6 部署与文档
- [x] `config/mid360s_config.json`、`deploy/`（cyclonedds.xml、g1d.env、3 个 systemd 单元、run_all.sh、install.sh）
- [x] `scripts/build_host.ps1`、`scripts/build_robot.sh`（修复 SDK 符号链接、Orin 架构参数、安装 + RPATH、测试）、`scripts/pack_for_robot.ps1`
- [x] README（构建、移植、运行、上线顺序、已知限制）、`docs/CALIBRATION.md`

## 4. 关键设计决策

| 决策 | 原因 |
|---|---|
| Nav2 代码用"原样 vendor + ros_shim 垫片 + 少量补丁" | 保持算法与上游一致，补丁集中、可追溯；便于后续合入上游更新 |
| FAST-LIO 用 `pcl_lite` 代替 PCL，IKFoM 去掉 Boost（只留 Boost.Preprocessor 头文件） | Windows 与 Jetson 都无需安装 PCL / Boost |
| 未修改的依赖用 git submodule，修改过的上游代码 vendor 进仓库 | submodule 保持可追溯；vendor 代码的每处修改都在本仓库 git 历史里 |
| C++ 核心与 DDS 适配分离 | 本机无法链接 unitree_sdk2，核心逻辑仍能完整测试 |
| guard 转发底盘扫描为 `PointCloud2_` | C++ 侧要用 `LaserScan_` 必须用 idlc 生成类型支持；转发后 C++ 只用 SDK 自带类型 |
| guard 反射层用"整车矩形沿指令圆弧扫掠" | 比固定扇区 / 直走廊更准：转向避让时不误停，向障碍转弯时提前停；慢速区加爬行速度下限，避免永远到不了停车线 |
| guard 启动时静默，只在运动或零速保持期内发布 | 不抢遥控器 / 其他控制源；停车后按 `zero_hold` 释放底盘 |
| 安装时把 `libddsc*.so` 以 SONAME 复制到 `install/lib/g1d` 并设 RPATH | SDK 里的 `.so.0` 是 git 符号链接，经 Windows 拷贝会变成文本文件 |

## 5. 进度日志

| 时间 | 进度 |
|---|---|
| 2026-11-13 | 环境确认：MinGW g++13.2、Python3.12、pip 安装 cmake/ninja/pytest/cyclonedds 11.1；unitree_sdk2py 在本机 DDS 回环测试通过。上游克隆到 `_upstream/`：FAST_LIO@7cc4175、FAST-LIO-NON-ROS、livox_ros_driver2、navigation2@4c01575（sparse） |
| 2026-11-13 | M0–M1：子模块、原样 vendor（FAST_LIO / Nav2 子集），ros_shim 让 Nav2 costmap / voxel / navfn / MPPI 原码编译通过 |
| 2026-11-13 | M2：FAST-LIO2 去 ROS / PCL / Boost，`LioEstimator`；合成 Mid-360 + IMU 仿真，6.9 m 轨迹最大误差 0.05 m |
| 2026-11-13 | M3：运动学、状态估计、代价地图层、NavFn、MPPI、NavCore；修复 Costmap2D 默认值未初始化、近目标停滞、末段旋转被堵等问题；8 个闭环场景通过 |
| 2026-11-13 | M4：`g1d_lio` / `g1d_nav` DDS 程序、Livox-SDK2 接入；`g1d_sim_demo` 演示（4 个场景） |
| 2026-11-14 | M4 收尾：SDK 头文件语法检查发现 `dds` 命名空间冲突并修复；`g1d_fk`；`rt/nav/livox_cloud` |
| 2026-11-14 | M5：guard（扫掠路径反射层 + 爬行速度下限，回环测试停在墙前 0.46 m）、NavClient、NavGPT 桥、mock 机器人、5 个现场工具、标定算法；pytest 28 项通过；多进程冒烟通过 |
| 2026-11-14 | M6：部署文件、构建 / 打包脚本（CMake 3.16 兼容、符号链接 / LF / 可执行位检查）、README、标定文档；全量构建 + 测试通过 |

## 6. 上机待办（本机无法完成）

- [ ] 在 Jetson 上 `scripts/build_robot.sh`：首次真正编译链接 `g1d_lio` / `g1d_nav`（gcc 9.4）
- [ ] P0：`check_interfaces.py`（确认 Mid-360S 数据源、SLAMWARE 话题、HTTP 事件格式）、`cmd_vel_probe.py`（`zero_hold`、限幅、锁轮）
- [ ] 标定：`head_to_livox`、`base_to_agv_z`、升降比例、自身点框、导航姿态（`docs/CALIBRATION.md`）
- [ ] 验收 T1–T6（README 第 6 节）；按结果调整 `guard.stop_m`、膨胀半径、MPPI 权重
