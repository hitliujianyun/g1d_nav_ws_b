# g1d_ws 项目详细介绍

## 1. 背景与目标

宇树 G1-D 是"人形上身 + 差速轮式底盘"的机器人。底盘可以通过 DDS 话题 `rt/cmd_vel_no_limit`（`linear.x` m/s、`angular.z` rad/s）直接控制，但原来的导航方式（NavGPT 给出航点 → 先转到目标角度、再直行）没有定位闭环，也没有避障；底盘自带的单线雷达只能看到约 0.1 m 高度的一个平面，看不到桌沿、货架隔板这类悬空障碍。

本项目在**不安装 ROS** 的前提下，用机器人头部的 **Livox Mid-360S** 实现：

1. **激光里程计**：FAST-LIO2 输出高精度位姿，并与底盘轮速融合，给出连续、低延迟的 `base_link` 位姿；
2. **三维避障导航**：体素代价地图 + 底盘雷达 + 膨胀，NavFn 全局规划，Nav2 MPPI 局部控制；
3. **独立安全守护**：所有到底盘的速度指令都经过 `g1d_base_guard` 的限幅、看门狗和近距离反射停车；
4. **NavGPT 接入**：保留"相对航点"的接口习惯，由导航系统做延迟补偿、绕障和失败反馈；
5. **调试与验收工具**：接口探测、底盘行为实测、标定、录制、Web 调试台和 T1–T6 验收向导。

运行环境：机器人 PC2（Jetson Orin NX，Ubuntu 20.04，aarch64），宇树 SDK2（CycloneDDS 0.10.x）。

## 2. 总体架构

```
                  ┌───────────── 机器人原有接口 ─────────────┐
Mid-360S ─UDP──▶ g1d_lio ── lio_odom / registered_cloud / lio_health ──┐
                                                                     ▼
slamware odom（轮速）、hispeed_state（升降）、lowstate / waistpitchstate（关节）──▶ g1d_nav ◀── rt/nav/goal ◀── NavGPT / Web / nav_cli
g1d_base_guard ── chassis_scan（底盘雷达转点云）──────────────────▶   │
                                                                     │ cmd_vel / status / debug / obstacle_points / costmaps
                                                                     ▼
slamware scan、wirelesscontroller、底盘 HTTP 事件、estop、teleop ──▶ g1d_base_guard ──▶ rt/cmd_vel_no_limit（唯一写入者）
                                                                     │
                                     g1d_web（浏览器调试台）◀── 订阅全部 rt/nav/* ──┘
```

| 进程 | 语言 | 频率 | 职责 | 建议 CPU 核 |
|---|---|---|---|---|
| `g1d_lio` | C++17 | 10 Hz（帧） | Mid-360S 取数、帧聚合、自身点过滤、FAST-LIO2、健康监测 | 2、3 |
| `g1d_nav` | C++17 | 20 Hz（控制） | 身体运动学、状态估计、代价地图、NavFn、MPPI、目标状态机 | 4、5 |
| `g1d_base_guard` | Python | 30 Hz | 底盘指令唯一写入者：限幅、看门狗、反射层、急停、遥控、底盘雷达转发 | 6 |
| `g1d_web` | Python | 10–20 Hz | Web 调试 / 可视化 / 控制 | 7 |

启动顺序：guard → web → lio（机器人静止 2 s 用于 IMU 初始化）→ nav。guard 最先启动、最后退出。

进程之间只通过 DDS 话题交互，任何一个进程崩溃都不会让底盘失控：nav 停止发指令后 guard 0.3 s 内减速停车；guard 本身停止后底盘依靠自身的指令超时停车。

## 3. 坐标系

| 坐标系 | 定义 |
|---|---|
| `odom` | 导航坐标系：原点为 LIO 启动时 `base_link` 的位置，z 轴竖直向上，高度以地面为 0。所有目标、路径、代价地图、Web 地图都在这个坐标系 |
| `odom_lio` | FAST-LIO2 的世界系（启动时 Mid-360S 内置 IMU 的位姿）。`T_odom_lio` 在第一帧 LIO 时由身体运动学锚定，`registered_cloud` 在此坐标系 |
| `base_link` | 底盘中心在地面的投影，x 前、y 左、逆时针为正。NavGPT 航点、障碍点、底盘雷达点都在此坐标系 |
| `head_link` | 头部，由 URDF 链 `base_link → AGV_link → 升降两级 → Pitching → 腰俯仰 → 腰偏航 → head_link` 计算 |
| `livox` / `livox_imu` | Mid-360S 雷达坐标系（倒装）与其内置 IMU。`head_to_livox` 需要标定，`livox → imu` 用 FAST-LIO 的 `mid360.yaml` 出厂值 |

头部雷达随升降、腰部运动，所以 `g1d_nav` 每个周期用关节反馈计算 `base_link → livox`，把 LIO 的 IMU 位姿换算成底盘位姿；导航时要求身体保持导航姿态（姿态锁定检查），避免把手臂运动误当成底盘运动。

## 4. 模块详解

### 4.1 g1d_lio：激光里程计

- **数据源**（`lidar.source`）：
  - `livox_sdk2`：Livox-SDK2（≥ 1.3.1 才支持 Mid-360S）直连雷达，读取 Cartesian High 点包和 IMU 包；IMU 加速度单位为 g；
  - `unitree_topic`：订阅宇树自带服务发布的 `rt/utlidar/cloud_livox_mid360` / `imu_livox_mid360`（雷达已被宇树服务占用时使用）。
- **帧聚合**：把点包按 `lidar.frame_ms`（100 ms）拼成一帧，每个点的时间 = 包时间戳 + 点序号 × 点间隔，供 FAST-LIO 去畸变。
- **预处理**：去掉 `blind`（0.6 m）以内和 `max_range` 以外的点；**自身点过滤**——把 `self_filter.boxes`（head_link 坐标系下的盒子：头、躯干、立柱、收拢的手臂）内的点删掉，避免机器人自身被当作障碍或特征。
- **FAST-LIO2**：迭代误差状态卡尔曼滤波（IESKF）+ ikd-Tree 增量地图，算法代码保持上游原样，只去掉了 ROS / PCL / Boost 依赖并封装为 `fast_lio_api`（`init / push_imu / push_scan / process / reset`）。
- **健康监测**：有效点数过少、单帧位移 / 转角跳变、处理超时、高度漂移、法向量特征值过小（退化场景，如长走廊）时报告错误；连续 `bad_frames_to_fail` 帧异常即 `ok=false`。
- **时钟映射**：雷达时间戳与主机时间不同步时，用滑动窗口最小偏移把雷达时间映射到主机时间（`ClockMapper`），输出的时间戳为主机时间。
- **输出**：`rt/nav/lio_odom`（IMU 在 `odom_lio` 下的位姿）、`rt/nav/registered_cloud`（配准后的当前帧点云，5 cm 降采样）、`rt/nav/lio_health`（JSON），可选 `rt/nav/livox_cloud`（雷达坐标系原始点云，标定用）。

### 4.2 g1d_nav：导航核心

**身体运动学（`BodyKinematics`）**：按 URDF 链和关节反馈（`hispeed_state.y` 升降高度、`lowstate` 12 号电机腰偏航、`waistpitchstate` 腰俯仰、15–28 号手臂）计算 `base_link → head → livox → imu`；`checkLock` 判断身体是否保持在导航姿态（腰部、升降、手臂偏差在 `lock_tolerance` 以内）。

**状态估计（`StateEstimator`）**：

- 第一帧可用的 LIO 位姿到来时锚定 `T_odom_lio`，之后 `base_link` 位姿 = `T_odom_lio · T_lio_imu · T_imu_base`；
- LIO 10 Hz 之间用轮速增量外推（最多 `max_extrapolation_s`），给 20 Hz 控制提供低延迟位姿；
- 检查 LIO 与轮速在 `consistency_window_s` 内的位移 / 转角一致性（打滑、LIO 漂移时报告 `odom_consistent=false`）；
- 保存 10 s 位姿历史，用于 NavGPT 目标的延迟补偿（`base_at_stamp`）；
- LIO 超时（`lio_timeout_s`）后默认判定定位丢失（`allow_wheel_only: false`）。

**代价地图**（Nav2 `Costmap2D` 原码 + 本项目的图层）：

| 地图 | 尺寸 / 分辨率 | 用途 |
|---|---|---|
| local | 6 × 6 m，0.05 m，随机器人滚动 | MPPI 局部避障 |
| global | 20 × 20 m，0.1 m，随机器人滚动 | NavFn 全局规划、目标合法性检查 |

- **VoxelLayer**：每个栅格存一列高度位掩码（`z_min` 0.08 m 到 `z_max` 1.85 m，0.1 m 一格）。Mid-360S 点云标记障碍，三维射线（Amanatides–Woo）清除视线经过的体素——只清除射线真正穿过的高度，所以桌面、货架隔板这样的**悬空障碍不会被地面方向的射线误清除**。`decay_s: 0` 表示障碍保留到被射线清除（雷达近距离盲区内仍然安全）。
- **ScanLayer**：底盘单线雷达（guard 转发的 `rt/nav/chassis_scan`）标记和二维射线清除，覆盖头部雷达看不到的近地低矮障碍。
- **InflationLayer**：Nav2 的膨胀公式（`radius` 0.55 m、`cost_scaling_factor` 3.0），实现 MPPI 的 `InflationLayerInterface`，CostCritic 可直接使用。

**全局规划（`GlobalPlanner`）**：NavFn（Dijkstra / A*），每 `1/replan_hz` 秒重规划。目标合法性检查（`sanitizeGoal`）：

- 超过 `max_goal_distance`（9 m）→ 截断到范围内，`partial=true`；
- 目标落在障碍或高代价区 → 沿目标→机器人方向回退，或在 `goal_search_radius` 内搜索最近可达点，`adjusted=true`；仍找不到 → `goal_in_obstacle`；
- 目标在全局代价地图之外 → `goal_out_of_range`。

**局部控制（`MppiController`）**：Nav2 MPPI（Eigen 版）原码，差速运动模型，56 步 × 0.05 s（2.8 s）预测，1000 条采样轨迹；评价项：Constraint、Cost（考虑整车轮廓）、Goal、GoalAngle、PathAlign、PathFollow、PathAngle、PreferForward。参数名与 Nav2 一致（`mppi:` 段）。

**目标状态机（`NavCore`）**：

| 状态 | 含义 |
|---|---|
| `IDLE` | 无目标，不输出速度（guard 自动归零） |
| `NAVIGATING` | 规划 + MPPI 跟踪；接近目标（`approach_distance` 0.6 m）后切换为直线末段接近；到达后若 `use_yaw` 则原地对齐朝向 |
| `ROTATING` | `rotate` 模式原地旋转（先检查旋转圆内是否有障碍） |
| `RECOVERING` | 恢复行为：`wait`（等待动态障碍离开）→ `clear`（清除 `recovery_clear_radius_m` 外的障碍）→ `rotate`（小角度原地转，刷新视野） |
| `SUCCEEDED` / `FAILED` / `CANCELED` | 结束状态，附带 `reason` |

- **模式**：`navigate`（规划 + MPPI）、`simple`（转向后直行，作为对照基线，不规划路径）、`rotate`、`stop`（取消当前目标）。
- **坐标系**：`base_at_stamp`（相对 `stamp` 时刻的 `base_link`，NavGPT 推荐）或 `odom`（绝对坐标，Web 点选和任务使用）。
- **进度检查**：`progress_window_s`（8 s）内距离没有减少 `progress_min_m`（0.2 m）→ 进入恢复；恢复用尽 → `FAILED: blocked`。
- **安全联动**：LIO 丢失、身体姿态变化、guard 急停、人工接管（遥控器或 Web 遥控）都会立即结束当前目标。
- **可通行扇区（`free_sectors`）**：−90°…+90° 每 30° 一个方向，检查前方 `free_sector_range_m`（1.5 m）内是否可通行，随状态发布，供 NavGPT 在失败后重新决策。

**输出**：`rt/nav/cmd_vel`（20 Hz，交给 guard）、`rt/nav/status`（10 Hz JSON）、`rt/nav/base_odom`、`rt/nav/obstacle_points`（`base_link` 下 3 m 内、高度带内的障碍点，供 guard 反射层）、`rt/nav/local_costmap` / `global_costmap`、`rt/nav/debug`（`T_odom_lio`、路径、MPPI 轨迹）、`rt/nav/param_state`。

### 4.3 g1d_base_guard：底盘安全守护

guard 是 `rt/cmd_vel_no_limit` 的**唯一写入者**（启动时用 DDS 发现检查是否有其他写入者，默认拒绝启动）。每个 30 Hz 周期：

1. **选择指令源**：急停或遥控器接管 → 0；Web 遥控（`rt/nav/teleop_cmd`）新鲜 → 用遥控（限速 `teleop_vx_max` / `teleop_wz_max`）；`rt/nav/cmd_vel` 在 `cmd_timeout`（0.3 s）内 → 用导航指令；否则 0。
2. **限幅**：`vx ∈ [vx_min, vx_max]`，`|wz| ≤ wz_max`。
3. **传感器健康**：底盘雷达超时，或 `require_cloud: true` 时 Mid-360S 障碍点超时（`sensor_timeout` 0.5 s）→ 只允许停车（`blocked: scan_stale / cloud_stale`）。
4. **前向反射层**：把整车矩形沿指令圆弧（`v, w`）向前扫掠，求到第一个障碍点（底盘雷达点 + 障碍点，去掉车身自身点）的距离 `d`：`d < stop_m`（0.45 m）→ 停车并禁止原地转（`blocked: front`）；`d < slow_m`（1.2 m）→ 线性减速，但不低于 `creep_v`（保证能走到停车线）。转弯避开障碍时不会误停，转向障碍时会提前停。
5. **后退保护**：后退时同样扫掠，后方 `rear_clear_m` 内有障碍 → 禁止后退（`blocked: rear`）。
6. **旋转保护**：原地旋转时，`rotate_radius`（0.38 m）圆内、车身外有障碍 → 禁止旋转（`blocked: rotate`）。
7. **急停锁存**：`rt/nav/estop "on"`、底盘 HTTP 事件 `BUMPER_TRIGGERED` / `CLIFF_DETECTED`（只认启动后新出现的事件）→ 立即归零并锁存，必须发 `"off"` 人工解除。
8. **遥控器接管**：`rt/wirelesscontroller` 摇杆超过死区 → `manual_hold_s`（5 s）内 guard 完全不发布，让遥控器优先。
9. **加速度限制**：加速 `acc_v`、减速 `brake_v`、角加速度 `acc_w`。
10. **发布策略**：运动中或归零后 `zero_hold` 时间内持续发布，之后停止发布，让底盘松开。

guard 还把底盘 `LaserScan_` 转成 `base_link` 下的点云发布到 `rt/nav/chassis_scan`（C++ 侧因此不需要为 `LaserScan_` 生成类型支持），并发布 `rt/nav/guard_state`（10 Hz JSON：拦截原因、急停、人工接管来源、前后净距等），支持 `rt/nav/param_set` 白名单参数热更新。

### 4.4 g1d_web：Web 调试台

运行在 PC2 上的 asyncio 服务（只用 Python 标准库 + numpy + unitree_sdk2py），浏览器端为原生 JavaScript + three.js r160（随仓库提供，离线可用）。功能：3D / 俯视显示累积点云地图、当前帧、障碍点、底盘雷达、代价地图、路径、MPPI 轨迹、可通行扇区；点选导航、多航点任务、Web 遥控、急停、T1–T6 验收向导、参数热更新与写回、地图导出（PCD、PNG + YAML、轨迹 CSV）。同一时间只有一个页面拥有控制权。详见 [`WEB_UI.md`](WEB_UI.md)。

### 4.5 工具与仿真

| 工具 | 用途 |
|---|---|
| `tools/check_interfaces.py` | 监听所有机器人 / 本项目话题，报告频率、时间戳偏差、样本内容、`cmd_vel_no_limit` 写入者、底盘 HTTP 连通性 |
| `tools/cmd_vel_probe.py` | 实测 `rt/cmd_vel_no_limit` 停发后的停车时间、零速刹车、限幅、零速是否锁轮 |
| `tools/calib_floor.py` + `g1d_fk` | 地面法标定雷达横滚 / 俯仰 / 高度，与底盘雷达对齐求偏航 |
| `tools/record.py` | 录制话题为 JSONL.gz（点云 base64） |
| `tools/nav_cli.py` | 命令行发目标、旋转、取消、急停、查看状态 |
| `tools/sim_stack.py` | 本机无硬件运行 fake_nav + mock 底盘 + 真实 guard + web |
| `g1d_sim_demo` + `tools/plot_sim_trace.py` | C++ 离线闭环仿真演示与轨迹图 |

`src/sim` 是 C++ 闭环仿真库（盒子世界、Mid-360 射线仿真、IMU 仿真、差速底盘），用于 LIO 精度测试和导航闭环测试；`python/g1d_nav_py/mock_robot.py` 和 `fake_nav.py` 在 DDS 层模拟底盘和 lio + nav，用于 Python / Web 的端到端测试。

## 5. 接口

### 5.1 DDS 话题

| 话题 | 类型 | 发布者 | 频率 | 说明 |
|---|---|---|---|---|
| `rt/slamware_ros_sdk_server_node/odom` | `nav_msgs::Odometry_` | 底盘 | ~20 Hz | 轮速里程计（时间戳比主机慢约 181 s，按接收时间处理） |
| `rt/slamware_ros_sdk_server_node/scan` | `sensor_msgs::LaserScan_` | 底盘 | 10 Hz | 底盘单线雷达，360 束 |
| `rt/hispeed_state` | `geometry_msgs::Point32_` | 本体 | — | `y` 升降高度，`z` 升降速度 |
| `rt/lowstate` / `rt/waistpitchstate` | `unitree_hg::LowState_` / `MotorState_` | 本体 | — | 腰偏航（12 号）、手臂（15–28 号）/ 腰俯仰 |
| `rt/wirelesscontroller` | `unitree_go::WirelessController_` | 本体 | — | 遥控器（人工接管检测） |
| `rt/nav/lio_odom` | `Odometry_` | g1d_lio | 10 Hz | IMU 在 `odom_lio` 下的位姿 |
| `rt/nav/registered_cloud` | `PointCloud2_` | g1d_lio | 10 Hz | 配准点云（`odom_lio`） |
| `rt/nav/lio_health` | `String_`（JSON） | g1d_lio | 10 Hz | `ok`、错误列表、有效点数、残差、处理耗时、时钟偏移 |
| `rt/nav/chassis_scan` | `PointCloud2_` | guard | 10 Hz | 底盘雷达点（`base_link`） |
| `rt/nav/goal` | `String_`（JSON） | 客户端 | 事件 | 导航目标（5.2 节） |
| `rt/nav/status` | `String_`（JSON） | g1d_nav | 10 Hz | 导航状态（5.3 节） |
| `rt/nav/cmd_vel` | `Twist_` | g1d_nav | 20 Hz | 送给 guard 的速度 |
| `rt/nav/base_odom` | `Odometry_` | g1d_nav | 20 Hz | 融合后的 `base_link` 位姿（`odom`） |
| `rt/nav/obstacle_points` | `PointCloud2_` | g1d_nav | 10 Hz | guard 反射层用的障碍点（`base_link`） |
| `rt/nav/local_costmap` / `global_costmap` | `OccupancyGrid_` | g1d_nav | 2 / 1 Hz | 调试显示 |
| `rt/nav/debug` | `String_`（JSON） | g1d_nav | 5 Hz | `T_odom_lio`、目标、全局 / 局部路径、MPPI 轨迹、限速 |
| `rt/nav/estop` | `String_` | 任意 | 事件 | `"on"` 急停锁存 / `"off"` 解除 |
| `rt/nav/teleop_cmd` | `Twist_` | g1d_web | 10 Hz | Web 遥控（deadman） |
| `rt/nav/param_set` / `param_state` | `String_`（JSON） | web / guard、nav | 事件 / 1 Hz | 参数热更新 |
| `rt/nav/guard_state` | `String_`（JSON） | guard | 10 Hz | 拦截原因、急停、人工接管、净距 |
| `rt/cmd_vel_no_limit` | `geometry_msgs::Twist_` | **仅 guard** | 30 Hz | 底盘速度指令 |

### 5.2 目标 JSON（`rt/nav/goal`）

```json
{"id": "navgpt-0042", "mode": "navigate", "frame": "base_at_stamp", "stamp": 1794554321.234,
 "x": 1.80, "y": -0.40, "yaw": 0.52, "use_yaw": true,
 "xy_tolerance": 0.20, "yaw_tolerance": 0.25, "max_speed": 0.35, "timeout_s": 60}
```

| 字段 | 默认 | 说明 |
|---|---|---|
| `id` | — | 目标标识，状态里原样返回 |
| `mode` | `navigate` | `navigate` / `simple` / `rotate` / `stop` |
| `frame` | `base_at_stamp` | `base_at_stamp`：相对 `stamp` 时刻车体（省略 `stamp` 表示当前）；`odom`：绝对坐标 |
| `stamp` | 0 | 主机时间（s），超过 `goal_stamp_max_age_s`（5 s）的旧时刻会被拒绝 |
| `yaw` / `use_yaw` | 省略 = 不对齐朝向 | 到达后是否原地对齐朝向 |
| `xy_tolerance` / `yaw_tolerance` | 0.2 m / 0.25 rad | 到达判定 |
| `max_speed` | 0 = 默认 | 本目标限速，不超过 `mppi.vx_max` |
| `timeout_s` | 120 | 超时失败 |

### 5.3 状态 JSON（`rt/nav/status`）

主要字段：`goal_id`、`state`、`reason`、`partial`、`adjusted`、`distance_remaining`、`yaw_error`、`goal`、`pose`、`velocity`、`free_sectors`、`lio_ok`、`state_valid`、`state_source`、`odom_consistent`、`body_locked`、`lock_reasons`、`guard{blocked, estop, manual}`、`recoveries`、`path_points`、`stamp`。

### 5.4 结束原因（`reason`）

| 原因 | 状态 | 含义 |
|---|---|---|
| `""` | SUCCEEDED | 到达 |
| `yaw_not_reached` | SUCCEEDED | 位置到达，但朝向对齐被障碍阻挡 3 s 以上 |
| `stop` / `manual_override` | CANCELED | 收到 stop 目标 / 遥控器或 Web 遥控接管 |
| `invalid_goal: …` | FAILED | 目标 JSON 无效 |
| `goal_stamp_unavailable` | FAILED | `stamp` 太旧或超出位姿历史 |
| `goal_out_of_range` / `goal_in_obstacle` | FAILED | 目标在地图外 / 在障碍内且附近无可达点 |
| `no_path` | FAILED | 没有可行路径（含恢复后仍无路径） |
| `blocked` | FAILED | 长时间没有进展，恢复行为用尽；或 simple / rotate 被挡 |
| `timeout` | FAILED | 超过 `timeout_s` |
| `lio_lost` | FAILED | LIO 不健康或超时 |
| `body_not_locked` / `body_moved` | FAILED | 开始时不在导航姿态 / 导航中身体关节变化 |
| `guard_estop` | FAILED | 急停 |

## 6. 安全设计

| 层 | 机制 | 触发 | 兜底 |
|---|---|---|---|
| L1 规划 | 代价地图 + MPPI（整车轮廓、膨胀 0.55 m） | 正常避障 | L2 |
| L2 guard 反射层 | 扫掠路径、后退、旋转圆 | 0.45 m 停车 / 1.2 m 减速 | L3–L5 |
| L3 guard 看门狗 | 指令 / 传感器超时 | 上游失联、传感器断流 | 停车 |
| L4 急停 | 话题、底盘碰撞 / 悬崖事件、Web 急停按钮 | 锁存，人工解除 | — |
| L5 底盘自身 | 指令超时、AEB | guard 失联 | — |
| 流程 | 姿态锁定、LIO 健康、单一写入者、人工接管优先 | — | — |

## 7. 开源代码复用

| 来源 | 版本 | 使用方式 |
|---|---|---|
| FAST_LIO（含 ikd-Tree、IKFoM） | `7cc4175` | vendor 到 `third_party/fast_lio`：先原样提交，再去掉 ROS / PCL / Boost、封装 API；IESKF 与 ikd-Tree 算法未改 |
| Nav2（costmap_2d 核心、voxel_grid、navfn、mppi_controller、nav2_core / nav2_util 头文件） | `4c01575` | vendor 到 `third_party/nav2`，代码原样，通过 `src/ros_shim` 垫片（rclcpp 日志 / 参数、消息结构体、pluginlib 静态注册等）免 ROS 编译 |
| Livox-SDK2、unitree_sdk2、unitree_sdk2_python、Eigen 3.4.0、yaml-cpp 0.8.0、Boost.Preprocessor | — | git 子模块，未修改 |
| unitree_robotskills `slamware_idl.py` | — | 复制 `LaserScan_` 定义（真机验证过的类型） |
| nlohmann/json、doctest、three.js r160 | — | 单文件库，原样随仓库提供 |

上游克隆放在 `_upstream/`（不入库），每处修改都在本仓库 git 历史中，说明见 `third_party/*/UPSTREAM.md`。

## 8. 构建与测试

CMake 选项：`G1D_BUILD_TESTS`（默认 ON）、`G1D_WITH_UNITREE`（DDS 程序，仅 Linux）、`G1D_WITH_LIVOX`（Livox-SDK2 直连）、`G1D_ARCH_FLAGS`（如 `-mcpu=cortex-a78ae`）。CMake 3.16 即可；算法核心在 Windows（MinGW）和 Linux 都能编译测试。

| 测试 | 内容 | 结果 |
|---|---|---|
| `test_common` / `test_lio_units` / `test_nav_units` | 几何、配置、时钟、帧聚合、预处理、运动学、代价地图、规划、MPPI | 通过 |
| `test_lio_sim` | 合成 Mid-360（5000 点 / 帧、IMU 200 Hz），6.9 m 轨迹 | 最大误差 0.05 m / 0.09°，单帧 7 ms |
| `test_nav_sim` | 9 个闭环场景：绕箱、桌面悬空、房间连续航点 + 延迟补偿、目标在障碍内、原地转、simple 通 / 堵、LIO 丢失、死路、调试输出与参数热更新 | 全部无碰撞（整机 0–1.75 m 高度检测） |
| pytest（50 项） | guard 逻辑、消息、标定算法、NavGPT 桥、Web 各模块；DDS 回环端到端（guard 停在墙前 0.46 m）；Web 端到端（导航、任务、参数、导出、遥控、急停） | 通过 |

C++ DDS 程序（`g1d_lio` / `g1d_nav`）依赖 unitree_sdk2 的 Linux 库，开发机上只做了语法检查，需要在机器人上编译。

## 9. 已知限制

- 不保存地图、不做重定位：每次启动 `odom` 原点为启动位置；Web 导出的地图只用于查看和记录。
- 动态行人只通过体素射线清除和 guard 反射层处理，没有运动预测。
- LIO 在长走廊、空旷大厅等几何退化场景可能失效（健康监测会报告并停止导航）。
- 若干参数（外参、身体链偏移、底盘指令行为）需要在真机上实测，见 [`DEPLOYMENT.md`](DEPLOYMENT.md)。
