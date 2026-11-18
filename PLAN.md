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

### M7 Web 调试台（设计：`docs/superpowers/specs/2026-11-14-web-debug-ui-design.md`，使用：`docs/WEB_UI.md`）
- [x] g1d_nav：`rt/nav/debug`（T_odom_lio、目标、路径、MPPI 轨迹）、`rt/nav/global_costmap`、`param_set` / `param_state` 热更新（+ `manager.max_speed`）；C++ 测试
- [x] guard：`rt/nav/teleop_cmd`（deadman、优先于导航、`manual_source: web`、仍过反射层）、参数白名单热更新；单测
- [x] `g1d_web` 后端：标准库 HTTP + WebSocket、DDS hub、控制锁、体素建图与导出（PCD / PNG + YAML / CSV / costmap）、任务运行器、T1–T6 验收、YAML 写回
- [x] 前端（原生 ES 模块 + 自带 three.js r160）：3D / 俯视视图与 13 个图层、导航 / 任务 / 遥控 / 建图 / 测试 / 参数 / 诊断 7 个标签页
- [x] `fake_nav` + `tools/sim_stack.py`：本机无硬件全链路（模拟 lio + nav + 底盘，真实 guard + web）
- [x] 测试：Web 单测 18 项 + Web 端到端 1 项（pytest 共 50 项）；浏览器实测导航、任务、T1、遥控、参数、导出
- [x] 部署：`deploy/systemd/g1d-web.service`、`run_all.sh` 启动 web

### M8 动态物体（方案：`docs/DYNAMIC_OBJECTS_STUDY.md` 第 0 节，实现结果见同文第 8 节）
- [x] P1 代价地图：体素层"视线穿透清除"（方向分箱最近回波 + 连续 N 帧证据，盲区半径 / 无回波方向不积累证据）、`DynamicLayer`（TTL）、`NavCore::onDynamicPoints`；单测 2 项
- [x] P1 仿真：`NavSimulation` 逐点标注回调（行人真值），`test_dynamic_sim` 3 个场景（拖影清除时间、行人横穿导航、视场下方障碍保持 30 s）
- [x] P2 M-detector（hku-mars @ 0645dab，GPL-2.0）：原样提交 → 去 ROS / OpenCV / TBB 移植；体素投票 + 膨胀代替 DBSCAN；倒装雷达下把 case-1 一致性窗口改为 0.15 m
- [x] P2 FAST-LIO 接入：EKF 后调用标注回调，动态点及邻域不入 ikd-Tree，插入延迟 3 帧做时间一致性；`delete_boxes()`、`map_points()`；LIO 仿真行人测试
- [x] P2 DDS：`registered_cloud` 增加 `dynamic` 字段、`rt/nav/map_delete`、`rt/nav/person_points`；lio_health 增加动态统计；SDK 头文件语法检查
- [x] P2 Web：动态点 / 人员点图层，动态点不进累积地图；关键帧录制（KTH / DUFOMap PCD 格式）；导出时 DUFOMap 清理（子进程，`g1d_web.mapclean`），无 dufomap 时射线清除
- [x] P3 `g1d_perception`：teleimager 相机源、YOLO-seg 后端、掩码 → 点云关联（深度门限 + 去地面）、确认跟踪、贴地安全的删除盒；节点 + 数据集录制；5090 工具：SAM 3 教师标注、YOLO-seg 训练导出、逐帧 DUFOMap 标签（场景流）
- [x] fake_nav 行人（`--pedestrians`、`--person-points`）；测试：C++ 6 套全过，pytest 67 项全过；浏览器实测 `sim_stack.py --pedestrians 2 --person-points`：动态点 / 人员点图层跟随行人，累积地图无拖影
- [ ] 上机：见第 6 节 M8 项

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
| 动态点只进短时动态层，不进体素层和 ikd-Tree；任何来源标为动态即按动态处理 | 漏检只会让人多留一会儿（仍被射线 / 穿透清除），误检只会少记一个静态点，都不影响避障安全 |
| 视线穿透清除只在"该方向有回波且都在体素后面"时积累证据 | 车身遮挡、视场外和盲区方向没有证据，障碍不会因看不见而被清掉 |
| M-detector 的聚类阶段换成体素投票 | 上游 DBSCAN 依赖完整 PCL；体素投票 + 一格膨胀在仿真里召回更高，且零误检 |
| DUFOMap 在子进程里跑 | Windows 上与 CycloneDDS 同进程时出现访问冲突；子进程同时也是 5090 上的离线工具 |
| 相机证据不进安全回路 | 相机延迟、标定误差都只影响"人消失得多快"，不影响停车与避障 |

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
| 2026-11-14 | M7：Web 调试台。需求确认：后端在 PC2、离线、笔记本浏览器、建图只做可视化 + 导出、6 类控制功能全部要。实现后端 / 前端 / fake_nav；浏览器实测：点选目标绕箱到达、3 航点任务 3/3、T1 停车净距 0.44 m、摇杆原地转、参数热更新、地图导出；修复：栅格导出空洞（膨胀一格）、fake_nav 未发 lio_odom |
| 2026-11-18 | M8：动态物体 P1–P3 实现。仿真结果：行人拖影 ≤ 0.85 s 清除；有标注时行人横穿导航用时 20.8 s（无标注 36.9 s）；M-detector 召回 0.67–0.74（真值位姿 0.78–0.90）、静态误检 ≈ 0；行人走廊地图残留减少 67–83 %；DUFOMap（d_p = 1）合成场景移除 95 % 拖影、墙体保留 99.5 %+。排查：倒装雷达下 M-detector 召回只有 0.2（地面落入一致性窗口）→ 窗口改 0.15 m；DUFOMap 与 CycloneDDS 同进程访问冲突 → 子进程；Web 导出改后台线程后暴露 WebSocket 关闭等待问题 → 限时 |

## 6. 上机待办（本机无法完成）

- [ ] 在 Jetson 上 `scripts/build_robot.sh`：首次真正编译链接 `g1d_lio` / `g1d_nav`（gcc 9.4）
- [ ] P0：`check_interfaces.py`（确认 Mid-360S 数据源、SLAMWARE 话题、HTTP 事件格式）、`cmd_vel_probe.py`（`zero_hold`、限幅、锁轮）
- [ ] 标定：`head_to_livox`、`base_to_agv_z`、升降比例、自身点框、导航姿态（`docs/CALIBRATION.md`）
- [ ] 验收 T1–T6（README 第 6 节，或 Web「测试」页）；按结果调整 `guard.stop_m`、膨胀半径、MPPI 权重
- [ ] Web 调试台接真机：确认 `rt/nav/debug` / 代价地图在真实 g1d_nav 下的显示，Jetson 上 g1d_web 的 CPU 占用（大地图时可调大 `web.map_voxel`）
- [ ] M8 P1：真人走过时看 Web 代价地图，拖影 ≤ 1 s 消失；低矮箱子靠近到视场下方后保持 ≥ 30 s
- [ ] M8 P2：Orin NX 上看 `lio_health.label_ms`（目标 < 20 ms，超了调小 `lio.dynamic.max_points`）；真人走过后用 `map_points` / 导出地图检查残留；`process_ms` 与定位误差对比关闭时
- [ ] M8 P2：录关键帧 → 导出，检查 DUFOMap 清理（Orin 上 `pip install dufomap` 是否有 aarch64 轮子，否则在 5090 上用 `g1d-mapclean` 离线清理）
- [ ] M8 P3：相机内参 / `head_to_camera` 标定、`latency_s` 测量；`--record` 录数据 → 5090 上 SAM 3 标注 → 训练 → Orin 上导出 TensorRT；验收：3 m 内召回 ≥ 95 %、< 30 ms/帧
