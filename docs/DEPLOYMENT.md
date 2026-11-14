# 移植到实机与参数设置

本文说明如何把 `g1d_ws` 从开发机移植到 G1-D 的 PC2（Jetson Orin NX，Ubuntu 20.04，aarch64），如何分阶段上线，以及上线前**必须设置 / 标定的参数**和常用可调参数。标定的具体操作见 [`CALIBRATION.md`](CALIBRATION.md)，项目原理见 [`PROJECT_OVERVIEW.md`](PROJECT_OVERVIEW.md)。

## 1. 目标环境

| 项目 | 要求 / 默认值 |
|---|---|
| 计算机 | G1-D PC2（开发者计算板），Jetson Orin NX，Ubuntu 20.04（JetPack 5.x），aarch64 |
| 网络 | 机器人内网 `192.168.123.0/24`：PC2 `192.168.123.164`，底盘 HTTP `192.168.123.163:1448`；Mid-360S 的 IP 以实测为准 |
| 编译工具 | gcc ≥ 9.4（C++17）、CMake ≥ 3.16、make 或 ninja |
| Python | 3.8（系统自带）+ numpy、PyYAML、cyclonedds 0.10.2（unitree_sdk2_python 依赖） |
| 账户 | systemd 单元默认用户 `unitree`、安装目录 `/opt/g1d_ws`（可改，见 9 节） |

不需要：ROS、PCL、Boost、Eigen 系统包（Eigen 3.4 随子模块提供，20.04 自带的 3.3.7 不满足要求）。

## 2. 打包与拷贝

**推荐用打包脚本**（开发机 Windows）：

```powershell
cd g1d_ws
git status                       # 先提交所有修改，脚本只打包已提交内容
powershell -ExecutionPolicy Bypass -File scripts\pack_for_robot.ps1
# → dist\g1d_ws-<commit>.tar.gz（约 22 MB，含全部子模块）
scp dist\g1d_ws-<commit>.tar.gz unitree@192.168.123.164:~
```

打包基于 `git archive`，有两个原因：

- 换行统一为 LF。Windows 工作区里的 CRLF 文件会让 bash 脚本报 `\r: command not found`。
- unitree_sdk2 里的 `libddsc.so.0`、`libddscxx.so.0` 是 git 符号链接，Windows 工作区中会变成 10 字节的文本文件；打包后仍是符号链接。

Linux 开发机也可以直接 `git clone --recursive` 到机器人上（机器人能联网时）。

**机器人上解压**：

```bash
tar xzf g1d_ws-<commit>.tar.gz
sudo mv g1d_ws /opt/ && sudo chown -R unitree:unitree /opt/g1d_ws
```

## 3. 编译

```bash
cd /opt/g1d_ws
scripts/build_robot.sh            # 选项：--deps（apt 安装依赖，需联网）、--no-livox、--no-tests、--jobs N
```

脚本步骤：

1. 检查子模块是否齐全（git 仓库且缺子模块时自动 `submodule update`）；
2. **修复 CycloneDDS 符号链接**（`.so.0` 为小文本文件时重建为链接）；
3. `cmake -DCMAKE_BUILD_TYPE=Release -DG1D_WITH_UNITREE=ON -DG1D_WITH_LIVOX=ON`，Orin 上自动加 `-DG1D_ARCH_FLAGS=-mcpu=cortex-a78ae`；
4. 编译并安装到 `/opt/g1d_ws/install`：`bin/g1d_lio`、`bin/g1d_nav`、`bin/g1d_fk`、`bin/g1d_sim_demo`，以及 `lib/g1d/libddsc.so.0`、`libddscxx.so.0`（可执行文件 RPATH 指向这里，不依赖系统库路径）；
5. 运行 ctest（5 个测试程序）和 pytest（未安装 cyclonedds 时只跑不依赖 DDS 的部分）。

手动编译（等价命令）：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DG1D_WITH_UNITREE=ON -DG1D_WITH_LIVOX=ON \
      -DG1D_ARCH_FLAGS="-mcpu=cortex-a78ae" -DCMAKE_INSTALL_PREFIX=$PWD/install
cmake --build build -j6 && cmake --install build
(cd build && ctest --output-on-failure)
```

| CMake 选项 | 默认 | 说明 |
|---|---|---|
| `G1D_WITH_UNITREE` | OFF | 编译 `g1d_lio` / `g1d_nav`（需要 Linux + unitree_sdk2） |
| `G1D_WITH_LIVOX` | OFF | 编译 Livox-SDK2 直连源；关闭后 `g1d_lio` 只能用 `lidar.source: unitree_topic` |
| `G1D_BUILD_TESTS` | ON | C++ 测试 |
| `G1D_ARCH_FLAGS` | 空 | 额外编译参数 |

首次完整编译耗时较长（Nav2 MPPI 和 FAST-LIO 是大的编译单元），内存紧张时用 `--jobs 4`。

## 4. Python 环境

```bash
python3 -m pip install --user numpy pyyaml pytest
python3 -c "import cyclonedds, unitree_sdk2py; print('ok')"     # 检查是否已有
```

如果机器人上还没有 `unitree_sdk2_python`，按其 README 安装：先编译 CycloneDDS 0.10.x 的 C 库，再安装 Python 包。

```bash
cd ~ && git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install && cmake --build . --target install
export CYCLONEDDS_HOME=~/cyclonedds/install
pip3 install -e /opt/g1d_ws/third_party/unitree_sdk2_python
```

本项目的 Python 代码不需要安装，`deploy/g1d.env` 已把 `python/` 和 `third_party/unitree_sdk2_python` 加入 `PYTHONPATH`。也可以 `pip3 install -e /opt/g1d_ws/python`，会得到 `g1d-base-guard`、`g1d-web` 等命令。

## 5. 网络与 DDS

- `system.net_if`：PC2 连接 `192.168.123.x` 的网卡名（`ip addr` 查看，常见为 `eth0`）。所有进程都用它初始化 DDS；开发机上可写 `auto`。
- `system.dds_domain`：真机为 0（`unitree_sim_isaaclab` 仿真为 1）。
- `deploy/cyclonedds.xml`：固定网卡并加大接收缓冲，由 `CYCLONEDDS_URI` 引用（`deploy/g1d.env` 已设置）。网卡名不是 `eth0` 时同步修改此文件。
- 若有防火墙，放行 DDS 组播（UDP 7400–7500）、Livox 端口（UDP 56100–56501）、Web 端口（TCP 8080）。

## 6. 分阶段上线

每个阶段通过后再进入下一阶段。所有命令在 `/opt/g1d_ws` 下执行；Web 调试台 `http://192.168.123.164:8080` 可以在 P1 之后全程使用。

### P0 接口确认（机器人不动）

```bash
python3 tools/check_interfaces.py --duration 5          # → logs/interfaces.json
```

逐项确认：

| 检查项 | 期望 | 不符时 |
|---|---|---|
| `slamware .../odom`、`.../scan` 有数据 | ~20 Hz / 10 Hz；时间戳偏差约 181 s（正常，系统按接收时间处理） | 检查 `net_if`、DDS 域 |
| `rt/hispeed_state`、`rt/lowstate`、`rt/waistpitchstate` | 有数据 | 没有 `rt/waistpitchstate`（旧版腰部）时，腰俯仰检查自动跳过；没有升降或 `lowstate` 时检查 DDS 连接 |
| `rt/utlidar/cloud_livox_mid360` | 有数据 → 宇树服务占用雷达，设 `lidar.source: unitree_topic`；无 → `livox_sdk2` | — |
| `cmd_vel_no_limit_writers` | 空 | 有其他写入者时先查明来源（guard 默认拒绝启动） |
| `chassis_http.ok` | true | 检查 `system.chassis_http` |

然后**停掉 guard**，实测底盘指令行为（机器人前方留 3 m 空地，有人看护）：

```bash
python3 tools/cmd_vel_probe.py --i-know-it-moves --tests silence,zero,clamp,hold
```

按结果设置 `guard.zero_hold`、确认 `mppi.vx_max` / `wz_max` 不超过底盘实际上限（见 7.1 节）。

### P1 安全底线（只用 guard + 底盘雷达）

```bash
# 配置：guard.require_cloud: false（此阶段没有 Mid-360S 障碍点）
deploy/run_all.sh --guard-only        # 只启动 guard 和 web
```

- Web「测试」页 T1：机器人正对墙 / 桌腿 / 人腿 2–3 m，以 0.3 m/s 前进，guard 拦停后净距 ≥ 0.40 m，10/10 次。
- T2：行驶中中断指令，≤ 0.6 s 停车。
- 用遥控器推杆确认 guard 让出控制（`guard_state.manual`）；拍底盘碰撞条确认急停锁存。
- 可用 `nav_cli.py simple` 测试旧的"转向 + 直行"方式经 guard 后的行为。

### P2 激光里程计

1. 按 [`CALIBRATION.md`](CALIBRATION.md) 完成身体链和 Mid-360S 外参标定（7.1 节的参数）。
2. 启动全部进程（`g1d_lio` 启动后机器人静止 2 s）：

   ```bash
   deploy/run_all.sh
   ```

3. 检查 `rt/nav/lio_health.ok = true`、`status.lio_ok = true`、`body_locked = true`；Web 地图应与环境一致、墙面不出现重影。
4. T3：在地面贴起点标记，遥控绕 10 m 矩形回到标记，误差 < 0.10 m、< 2°。
5. T4：原地旋转 3 圈，平移漂移 < 0.05 m（外参偏差会表现为绕圈）。

### P3–P4 代价地图与导航

1. 恢复 `guard.require_cloud: true`。
2. Web 打开「局部代价地图」「障碍点」图层，确认桌沿、椅子靠背等悬空物在代价地图中可见，机器人自身没有出现在障碍点中（否则调大 `self_filter.boxes`）。
3. 点选短距离目标（1–2 m），逐步加长；再测试绕障。
4. T5：走廊放纸箱、椅子，绕障到达 ≥ 9/10；T6：桌面悬空物，10/10 无碰撞完成或安全拒绝。
5. 接入 NavGPT（`NavGptBridge`），设置 `camera_latency_s` 为实测的相机延迟。

### 部署为服务

```bash
sudo deploy/install.sh --enable
sudo systemctl start g1d-guard g1d-web g1d-lio g1d-nav
journalctl -fu g1d-nav                          # 查看日志
```

## 7. 需要设置的参数

所有参数在 `config/g1d_nav.yaml`（所有进程共用）。标 **必须** 的项在真机上一定要实测，否则定位或安全距离会有系统误差。

### 7.1 必须实测 / 标定

| 参数 | 当前值 | 含义 | 如何得到 |
|---|---|---|---|
| `kinematics.head_to_livox.xyz / rpy` | `[0.05, 0, 0.55]` / `[π, −0.04, 0]` | head_link → Mid-360S 外参（倒装） | `g1d_fk` + `tools/calib_floor.py`（地面法求横滚、俯仰、高度，与底盘雷达对齐求偏航） |
| `kinematics.base_to_agv_z` | 0.0 | AGV_link 原点离地高度 | 卷尺测量，或由 `calib_floor.py` 高度误差反推 |
| `kinematics.lift_scale / lift_offset` | 1.0 / 0.0 | 升降总行程 = `lift_scale · hispeed_state.y + lift_offset` | 最低 / 最高位各读一次 `hispeed_state.y` 并测量实际高度 |
| `kinematics.nav_pose.waist_pitch` | 0.0 | 导航姿态的腰俯仰（rad） | 摆好导航姿态后读 `rt/waistpitchstate.q` |
| `kinematics.nav_pose.arm_tuck` | `[]`（不检查手臂） | 导航姿态的 14 个手臂关节（15–28 号电机） | 摆好姿态后读 `check_interfaces.py` 中 `lowstate.arms_q15_28` |
| `self_filter.boxes` | 两个估计盒子 | head_link 下的自身点盒子 `[xmin, xmax, ymin, ymax, zmin, zmax]` | 静止时 Web 查看障碍点 / `livox_cloud`，车身周围 0.3 m 内不应有障碍点 |
| `robot.chassis_laser.xyz / rpy` | URDF 值 | 底盘雷达安装位姿（base_link） | 一般用 URDF；偏航可与 Mid-360S 对比确认 |
| `guard.zero_hold` | 1.0 s | 归零后继续发布零速的时间 | `cmd_vel_probe.py` 的 `silence.stop_time_s` + 0.3 s（工具给出建议值） |
| `mppi.vx_max / wz_max` | 0.40 / 0.60 | 导航最大线 / 角速度 | 不超过 `cmd_vel_probe.py clamp` 实测的底盘上限 |

### 7.2 按现场确认

| 参数 | 默认 | 说明 |
|---|---|---|
| `system.net_if` | `eth0` | PC2 内网网卡名；同步修改 `deploy/cyclonedds.xml` |
| `system.dds_domain` | 0 | 真机 0 |
| `system.chassis_http` | `http://192.168.123.163:1448` | SLAMWARE 底盘 HTTP（事件、终止任务） |
| `lidar.source` | `livox_sdk2` | `livox_sdk2` 直连 / `unitree_topic` 订阅宇树话题（P0 判定） |
| `lidar.host_ip` + `config/mid360s_config.json` 的 `host_ip` | `192.168.123.164` | 雷达把数据发往的主机地址，两处保持一致 |
| `lidar.unitree_cloud_topic / unitree_imu_topic` | `rt/utlidar/...mid360` | `unitree_topic` 模式下的话题名 |
| `guard.require_cloud` | true | P1 阶段设 false（只用底盘雷达）；导航阶段必须为 true |
| `guard.other_writers` | `refuse` | 有其他 `cmd_vel_no_limit` 写入者时拒绝启动；确认无冲突后才可改 `warn` |
| `kinematics.require_lock` | true | 导航时要求身体保持导航姿态；调试阶段可暂时关闭 |
| `web.port` | 8080 | Web 调试台端口 |

### 7.3 常用可调参数

**guard（安全守护，可在 Web「参数」页热更新）**

| 参数 | 默认 | 说明 / 调整建议 |
|---|---|---|
| `vx_max` / `vx_min` / `wz_max` | 0.50 / −0.10 / 0.60 | 底盘指令硬限幅（应 ≥ MPPI 限速） |
| `acc_v` / `brake_v` / `acc_w` | 0.5 / 1.5 / 1.0 | 加速、减速（m/s²）、角加速度（rad/s²） |
| `stop_m` / `slow_m` | 0.45 / 1.20 | 扫掠净距停车线 / 开始减速距离；T1 不达标时加大 `stop_m` |
| `creep_v` | 0.05 | 减速区最低速度，保证能走到停车线 |
| `rear_clear_m` | 0.50 | 后退所需后方净距 |
| `front_x` / `half_width` | 0.28 / 0.30 | 扫掠矩形前沿、半宽（车体 557 × 525 mm + 余量） |
| `body_x` / `body_y` | 0.30 / 0.28 | 自身点盒子（盒内点忽略） |
| `rotate_radius` | 0.38 | 原地旋转保护圆半径 |
| `cmd_timeout` / `sensor_timeout` | 0.3 / 0.5 s | 指令 / 传感器超时 |
| `manual_deadzone` / `manual_hold_s` | 0.15 / 5.0 s | 遥控器接管判定 / 接管后静默时间 |
| `teleop_timeout` / `teleop_vx_max` / `teleop_wz_max` | 0.3 s / 0.30 / 0.50 | Web 遥控 deadman 与限速 |

**导航（manager，可热更新的项标 ★）**

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_speed` ★ | 0（= `mppi.vx_max`） | 默认目标限速，现场初期建议 0.2–0.3 |
| `approach_distance` / `approach_v_max` ★ | 0.6 m / 0.2 | 末段直线接近的距离（需重启）/ 速度 |
| `final_rotation_kp` ★ / `final_rotation_wmax` ★ | 1.5 / 0.5 | 到达后朝向对齐 |
| `simple_v_max` ★ / `simple_kp_yaw` | 0.3 / 1.5 | simple 模式 |
| `progress_window_s` ★ / `progress_min_m` ★ | 8 s / 0.2 m | 无进展判定（进入恢复） |
| `recovery` | `[wait, clear, rotate]` | 恢复行为顺序 |
| `recovery_wait_s` ★ / `recovery_clear_radius_m` ★ / `recovery_rotate_rad` ★ | 2 s / 1.5 m / 0.52 rad | 恢复参数 |
| `goal_stamp_max_age_s` | 5 s | `base_at_stamp` 目标允许的最大时间差（NavGPT 推理很慢时加大，但不超过位姿历史 10 s） |
| `free_sector_range_m` | 1.5 m | 可通行扇区检查距离 |
| `require_lio` | true | LIO 不可用时禁止导航 |

**代价地图（`costmap`）**

| 参数 | 默认 | 说明 |
|---|---|---|
| `common.voxel.z_min / z_max` | 0.08 / 1.85 m | 参与避障的高度带：低于 `z_min` 视为地面；`z_max` 应覆盖头部、手臂最高点 |
| `common.voxel.obstacle_max_range / raytrace_max_range` | 5 / 6 m | 标记 / 清除距离 |
| `common.voxel.decay_s` | 0 | 障碍衰减时间；0 = 只靠射线清除（盲区内更安全）；动态环境多时可设 3–5 s |
| `common.inflation.radius / cost_scaling_factor` | 0.55 m / 3.0 | 膨胀半径 / 代价下降速度；T5 / T6 离障碍太近时加大半径 |
| `local.size_x / size_y / resolution` | 6 m / 0.05 m | 局部地图 |
| `global.size_x / size_y / resolution` | 20 m / 0.1 m | 全局地图；目标距离上限见 `planner.max_goal_distance` |

**规划（`planner`）**：`replan_hz` 1.0、`allow_unknown` true（允许穿过未探测区域）、`tolerance` 0.3 m、`goal_search_radius` 0.5 m（障碍内目标外移搜索半径）、`goal_max_cost` 128、`max_goal_distance` 9 m。

**MPPI（`mppi`，Nav2 参数名）**

| 参数 | 默认 | 说明 |
|---|---|---|
| `rate_hz` / `model_dt` | 20 / 0.05 | 控制频率，必须满足 `rate_hz = 1 / model_dt` |
| `time_steps` / `batch_size` | 56 / 1000 | 预测步数（2.8 s）/ 采样数；CPU 不足时把 `batch_size` 降到 500–700 |
| `vx_max` / `vx_min` / `wz_max` | 0.40 / −0.10 / 0.60 | 速度上限（≤ guard 限幅，≤ 底盘实测上限） |
| `ax_max` / `ax_min` / `az_max` | 0.5 / −1.0 / 1.0 | 加速度约束 |
| `CostCritic.cost_weight` | 3.81 | 离障碍远近的权重；擦边太近时加大 |
| `PathAlignCritic.cost_weight` / `PathFollowCritic.cost_weight` | 14 / 5 | 贴合路径的程度 |
| `PreferForwardCritic.cost_weight` | 5 | 抑制倒车 |

**LIO（`lio`）**

| 参数 | 默认 | 说明 |
|---|---|---|
| `blind` | 0.6 m | 近距离盲区，过滤机器人自身与近处噪点 |
| `filter_size_surf / filter_size_map` | 0.3 / 0.4 m | 当前帧 / 地图降采样；室内小空间可降到 0.2 / 0.3 |
| `det_range` | 60 m | 地图保留范围 |
| `gyr_cov / acc_cov` | 0.1 / 0.1 | IMU 噪声 |
| `extrinsic_T / extrinsic_R` | Mid-360 出厂值 | 雷达与内置 IMU 外参，一般不改 |
| `health.min_effective_points` | 300 | 有效匹配点下限（退化检测） |
| `health.max_step_translation_m / max_step_yaw_deg` | 0.3 m / 15° | 单帧跳变上限 |
| `health.max_process_time_s` | 0.15 s | 单帧处理时间上限 |

**状态估计（`state`）**：`wheel_timeout_s` 0.3、`lio_timeout_s` 0.5、`max_extrapolation_s` 0.3、`consistency_window_s` 2.0、`max_lio_wheel_translation_m` 0.3、`max_lio_wheel_yaw_deg` 10、`use_wheel_odom` true、`allow_wheel_only` false（LIO 丢失时不允许只用轮速导航）。

**Web（`web`）**：`map_voxel` 0.05 m（地图很大、Jetson 负载高时加大到 0.1）、`map_max_points` 300 万、`scan_max_points` 3 万、`grid_z_min / grid_z_max` 0.10 / 1.80 m（导出栅格的占据高度带）。

**调试（`debug`）**：`publish_costmap` true；`publish_lidar_cloud` false（标定时打开）。

### 7.4 修改与保存参数

- **热更新**：Web「参数」页修改立即生效（guard 白名单参数和带 ★ 的导航参数），也可发 `rt/nav/param_set`。
- **写回配置**：Web「保存」写回 `config/g1d_nav.yaml`，保留注释并备份为 `g1d_nav.yaml.bak-<时间>`。
- **其他参数**修改 YAML 后重启对应进程（`sudo systemctl restart g1d-nav` 等）。
- **现场配置与代码分离**：重新打包、解压会覆盖 `config/`。建议把现场配置另存（如 `/opt/g1d_site.yaml`），在 `deploy/g1d.env` 中设置 `G1D_CONFIG=/opt/g1d_site.yaml`（systemd 使用）；手动运行时用 `G1D_CONFIG_OVERRIDE=/opt/g1d_site.yaml deploy/run_all.sh`。

## 8. 升级与回滚

1. 开发机提交修改后重新打包；
2. 机器人上 `sudo systemctl stop g1d-nav g1d-lio g1d-web g1d-guard`；
3. 把旧目录改名备份（`mv /opt/g1d_ws /opt/g1d_ws.old`），解压新包，`scripts/build_robot.sh`；
4. 现场配置用 7.4 节的方式保留；
5. 启动服务；有问题时停服务、恢复旧目录即可回滚。

## 9. 修改安装位置 / 用户

systemd 单元和 `deploy/g1d.env` 默认 `/opt/g1d_ws`、用户 `unitree`。换位置时修改 `deploy/g1d.env`（`G1D_ROOT`、`G1D_CONFIG`、`CYCLONEDDS_URI`、`PYTHONPATH`）和 `deploy/systemd/*.service` 里的路径与 `User=`，再执行 `sudo deploy/install.sh`。CPU 绑核（`CPUAffinity`）按 [`PROJECT_OVERVIEW.md`](PROJECT_OVERVIEW.md) 第 2 节的建议，可按实际负载调整。

## 10. 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 程序启动报 `libddsc.so.0: file too short` / 找不到 | SDK 符号链接变成了文本文件 | 用 `install/bin` 下的程序，或重跑 `scripts/build_robot.sh`（会修复链接） |
| 收不到任何话题 | `net_if` 错误、DDS 域不一致、防火墙 | `ip addr` 核对网卡；`check_interfaces.py --net-if <网卡>` |
| guard 启动后立即退出，日志 `other writers on rt/cmd_vel_no_limit` | 有其他进程在写底盘速度 | 停掉该进程（旧导航程序、示例程序）；确认无冲突后才可用 `--other-writers warn` |
| guard `blocked: cloud_stale` | `g1d_nav` 未运行或 Mid-360S 断流 | P1 阶段设 `guard.require_cloud: false`；否则检查 lio / nav |
| guard `blocked: scan_stale` | 底盘雷达话题中断 | 检查 SLAMWARE 服务 |
| `lio_health.ok=false`（`errors` 含 `few_points` / `jump_translation` / `jump_yaw` / `z_drift` / `slow`，或 `degenerate=true`） | 雷达被遮挡、自身点框过大、几何退化、CPU 不足 | 检查 `self_filter.boxes`、`blind`；避开长走廊等退化环境 |
| LIO 启动后漂移 | 启动时机器人在动 | 启动 `g1d_lio` 时保持静止 2 s；必要时重启 lio 和 nav |
| Livox 无数据（`livox_sdk2`） | 宇树服务占用雷达端口、`host_ip` 不对 | 改用 `unitree_topic`；核对 `mid360s_config.json` |
| 目标立即 `FAILED: body_not_locked` | 腰部未回零、升降在动、手臂不在收拢位 | 摆好导航姿态；核对 `nav_pose`，调试时可暂设 `require_lock: false` |
| `FAILED: body_moved` | 导航中升降或手臂动作 | 导航期间不要动上身；或放宽 `lock_tolerance` |
| `FAILED: goal_stamp_unavailable` | NavGPT 的 `stamp` 太旧（> 5 s）或时钟不同源 | 用 PC2 主机时间；推理太慢时加大 `goal_stamp_max_age_s`（≤ 10 s） |
| `FAILED: goal_in_obstacle` / `goal_out_of_range` | 目标在障碍内 / 超出全局地图 | 换目标；远目标拆成多个航点 |
| 频繁 `FAILED: blocked` | 膨胀过大、通道太窄、动态障碍 | 检查代价地图；适当减小膨胀半径或 `stop_m`（不低于 T1 要求） |
| 原地转圈后位置漂移（T4 不过） | Mid-360S 外参不准 | 重新做 `calib_floor.py` 标定 |
| 机器人自身出现在代价地图里 | 自身点框太小 | 加大 `self_filter.boxes` |
| Web 建图页显示"等待 T_odom_lio" | `g1d_nav` 未运行 | 启动 nav；LIO 重启后需「重置地图」 |
| Jetson CPU 占用过高 | MPPI 采样多、Web 地图大 | 降低 `mppi.batch_size`；加大 `web.map_voxel`；不用时关闭 Web |

## 11. 上线检查清单

- [ ] 打包版本对应的 commit 已记录
- [ ] `build_robot.sh` 编译通过，ctest / pytest 通过
- [ ] `check_interfaces.py` 各话题正常，无其他 `cmd_vel_no_limit` 写入者
- [ ] `cmd_vel_probe.py` 实测完成，`guard.zero_hold`、`mppi.vx_max / wz_max` 已按结果设置
- [ ] 身体链、Mid-360S 外参、自身点框、导航姿态已标定（7.1 节）
- [ ] T1 / T2 通过（guard）
- [ ] T3 / T4 通过（LIO）
- [ ] `guard.require_cloud` 恢复为 true
- [ ] T5 / T6 通过（导航）
- [ ] 现场配置另存，并在 `deploy/g1d.env` 中指向它
- [ ] systemd 服务已启用，重启机器人后自动运行
