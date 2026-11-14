# g1d_ws：G1-D 头部 Mid-360S 激光里程计 + 避障导航（不依赖 ROS）

设计文档：`../design/G1D_LIO_MPPI_Nav_Plan.md`。开发计划、TODO 和进度：[`PLAN.md`](PLAN.md)。

## 1. 组成

```
Mid-360S ──UDP──▶ g1d_lio (C++) ──rt/nav/lio_odom, registered_cloud, lio_health──┐
                                                                                 ▼
slamware odom / hispeed / lowstate / waistpitch ──────────────▶ g1d_nav (C++)  ◀── rt/nav/goal (NavGPT / nav_cli)
g1d_base_guard ──rt/nav/chassis_scan (底盘雷达转点云)──────────▶   FK + 状态估计 + 体素/扫描代价地图
                                                                  + NavFn + MPPI + 状态机
                                                                     │ rt/nav/cmd_vel, status, obstacle_points
                                                                     ▼
slamware scan, wirelesscontroller, HTTP 事件 ──▶ g1d_base_guard (Python) ──▶ rt/cmd_vel_no_limit（唯一写入者）
```

| 进程 | 语言 | 内容 |
|---|---|---|
| `g1d_lio` | C++17 | Livox-SDK2 直连 Mid-360S（或订阅 `rt/utlidar/*`）→ 帧聚合 + 自身点过滤 → FAST-LIO2（去 ROS / PCL）→ DDS |
| `g1d_nav` | C++17 | 身体 FK、LIO + 轮速融合、Nav2 Costmap2D（体素层保留悬空物 + 底盘扫描层 + 膨胀）、NavFn、Nav2 MPPI（Eigen 版）、目标状态机（navigate / rotate / simple / stop、延迟补偿、恢复行为、free_sectors） |
| `g1d_base_guard` | Python | 限幅、加速度限制、看门狗、扫掠路径反射层、旋转圆保护、急停锁存（话题 / 底盘 HTTP 事件）、遥控接管、零速保持后释放底盘 |
| `NavClient` / `tools/` | Python | 目标下发与等待、接口探测、`cmd_vel_no_limit` 行为实测、录制、外参标定 |

## 2. 目录

| 路径 | 说明 |
|---|---|
| `src/common` `src/lio` `src/nav` | 算法核心（纯 C++，Windows / Linux 都能编译测试） |
| `src/ros_shim` `src/pcl_lite` | 让 Nav2 / FAST-LIO 原码免 ROS / PCL 编译的最小垫片头文件 |
| `src/apps` | `g1d_lio` / `g1d_nav` DDS 可执行程序（仅 Linux + unitree_sdk2） |
| `src/sim` `src/tools` | 闭环仿真库；`g1d_sim_demo`（离线演示）、`g1d_fk`（身体链打印） |
| `third_party/fast_lio` `third_party/nav2` | vendor 的上游代码（先原样提交再修改，改动见各自 `UPSTREAM.md` 与 git 历史） |
| `third_party/<submodule>` | eigen 3.4.0、yaml-cpp、Livox-SDK2、unitree_sdk2、unitree_sdk2_python、boost_preprocessor |
| `python/g1d_nav_py` | guard、NavClient、mock 机器人、消息工具、标定算法；`python/tests` 为 pytest |
| `tools/` | 现场工具脚本（见第 6 节） |
| `config/` | `g1d_nav.yaml`（所有进程共用）、`mid360s_config.json`（Livox-SDK2） |
| `deploy/` | `cyclonedds.xml`、`g1d.env`、systemd 单元、`run_all.sh`、`install.sh` |
| `scripts/` | `build_host.ps1`（本机）、`build_robot.sh`（机器人 / Linux）、`pack_for_robot.ps1`（打包） |

## 3. 本机开发与验证（Windows）

依赖：MinGW-w64 g++ ≥ 13、CMake ≥ 3.16、Ninja、Python 3.8+（`pip install numpy pyyaml pytest cyclonedds matplotlib`）。

```powershell
git submodule update --init --recursive
powershell -ExecutionPolicy Bypass -File scripts\build_host.ps1 -Demo
```

会做：编译全部 C++ 核心 → `ctest`（5 个测试程序）→ `pytest`（28 个用例，含真实 CycloneDDS 回环端到端）→ 运行 4 个离线闭环场景并画图到 `logs/sim_*.png`。

本机验证覆盖范围：

| 内容 | 结果（当前提交） |
|---|---|
| FAST-LIO2 合成 Mid-360（5000 点/帧、IMU 200 Hz，6.9 m 椭圆轨迹） | 最大位置误差 0.05 m，偏航误差 < 0.1°，单帧 7 ms |
| 导航闭环 8 个场景（绕箱、桌面悬空、房间连续航点 + 延迟补偿、目标在障碍内、原地转、simple 通/堵、LIO 丢失、死路） | 全部无碰撞（整机高度 0–1.75 m 检测），状态 / 原因符合预期 |
| guard 逻辑 14 个单测 + DDS 回环（mock 底盘 ↔ guard ↔ 模拟 nav） | 0.3 m/s 驶向墙，停在 0.46 m（T1 要求 ≥ 0.40 m）；超时释放、原地转、后退、话题急停、HTTP 碰撞事件锁存均通过 |
| C++ DDS 程序 | 本机不能链接 unitree_sdk2（只有 Linux 库），已用 `-fsyntax-only` 对 SDK 头文件做语法检查；真正编译在机器人上完成 |

## 4. 移植到机器人（Jetson Orin NX，Ubuntu 20.04，aarch64）

1. **打包**（本机）：`powershell -ExecutionPolicy Bypass -File scripts\pack_for_robot.ps1`
   生成 `dist/g1d_ws-<commit>.tar.gz`。打包基于 `git archive`，换行为 LF，SDK 里的 `.so.0` 符号链接保持为链接（直接拷贝 Windows 工作区会丢失这两点）。
2. **拷贝并编译**（机器人 PC2，`192.168.123.164`）：
   ```bash
   scp dist/g1d_ws-<commit>.tar.gz unitree@192.168.123.164:~
   tar xzf g1d_ws-<commit>.tar.gz && sudo mv g1d_ws /opt/ && sudo chown -R unitree /opt/g1d_ws
   /opt/g1d_ws/scripts/build_robot.sh            # 首次联网可加 --deps
   ```
   脚本会：检查子模块 → 修复 CycloneDDS 符号链接 → `cmake -DG1D_WITH_UNITREE=ON -DG1D_WITH_LIVOX=ON`（Orin 上自动加 `-mcpu=cortex-a78ae`）→ 安装到 `/opt/g1d_ws/install`（`bin/g1d_lio`、`bin/g1d_nav`、`bin/g1d_fk`、`lib/g1d/libddsc*.so.0`，RPATH 已设置）→ 运行 ctest / pytest。
   - CMake 3.16（20.04 自带）即可；gcc 9.4 按 C++17 编译。
3. **Python 依赖**：`python3 -m pip install --user numpy pyyaml pytest`；`unitree_sdk2_python` 按其 README 安装（需要 CycloneDDS 0.10.2 的 C 库并设置 `CYCLONEDDS_HOME`，然后 `pip3 install -e third_party/unitree_sdk2_python`）。不安装也可以，`deploy/g1d.env` 已把源码目录加入 `PYTHONPATH`，但 `cyclonedds==0.10.2` 的 Python 包必须可用。
4. **Mid-360S 数据源**：先运行 `tools/check_interfaces.py`。
   - 若 `rt/utlidar/cloud_livox_mid360` 有数据，说明宇树自带服务已占用雷达：设 `lidar.source: unitree_topic`（此时不要再用 SDK 直连，否则端口冲突）。
   - 否则用 `lidar.source: livox_sdk2`，确认 `config/mid360s_config.json` 的 `host_ip` 是 PC2 的地址。

## 5. 运行

```bash
cd /opt/g1d_ws
deploy/run_all.sh --guard-only      # 第 1 阶段：只开 guard（底盘雷达反射层），用 nav_cli simple 测试
deploy/run_all.sh                   # guard → lio（静止 2 s）→ nav
# 或 systemd：
sudo deploy/install.sh --enable && sudo systemctl start g1d-guard g1d-lio g1d-nav
```

发目标：

```bash
python3 tools/nav_cli.py goal 1.5 0.0 --yaw 0.0 --wait        # 相对当前 base_link
python3 tools/nav_cli.py goal 3.0 1.0 --frame odom --wait     # 里程计系绝对坐标
python3 tools/nav_cli.py rotate 1.57 --wait
python3 tools/nav_cli.py stop ; python3 tools/nav_cli.py estop on|off
python3 tools/nav_cli.py watch [--guard]
```

NavGPT 侧（替换原来的"转到固定角度再直行"）：

```python
from g1d_nav_py.nav_client import NavClient
from g1d_nav_py.navgpt_bridge import NavGptBridge

bridge = NavGptBridge(NavClient(0, "eth0"), camera_latency_s=0.08, max_speed=0.35)
result = bridge.step(lambda: navgpt3.infer(image, instruction), t_recv=frame_recv_time)  # (dx, dy, dyaw)
prompt += bridge.feedback(result)        # FAILED 时带上 reason 与可通行方向
```

`dx, dy, dyaw` 是图像时刻 `base_link` 下的相对航点（x 前、y 左、逆时针为正），`g1d_nav` 用位姿历史补偿推理延迟。底层接口 `NavClient.send_goal / rotate / stop / wait` 与 JSON 格式见设计文档 7.2 / 7.3 节。

## 6. 现场工具与上线顺序（设计文档第 15 节）

| 步骤 | 命令 | 产出 / 通过标准 |
|---|---|---|
| P0 接口确认 | `python3 tools/check_interfaces.py` | `logs/interfaces.json`：各话题频率、SLAMWARE 时间戳偏差（约 181 s）、`rt/cmd_vel_no_limit` 已有写入者、底盘 HTTP |
| P0 底盘行为 | 停掉 guard，`python3 tools/cmd_vel_probe.py --i-know-it-moves` | 停发后多久停车、是否截断速度、零速是否锁住 → 填 `guard.zero_hold` 等 |
| 外参 | `install/bin/g1d_fk` + `tools/calib_floor.py`（见 [`docs/CALIBRATION.md`](docs/CALIBRATION.md)） | `kinematics.head_to_livox`、`base_to_agv_z`、自身点框 |
| P1 安全底线 | `run_all.sh --guard-only`，`guard.require_cloud: false` | T1：0.3 m/s 驶向墙 / 桌 / 人腿停车 ≥ 0.40 m；T2：杀进程 ≤ 0.6 s 停车 |
| P2 LIO | `run_all.sh`，`nav_cli watch` | T3：10 m 矩形闭环误差 < 0.10 m / 2°；T4：原地 3 圈漂移 < 0.05 m |
| P3–P4 导航 | `nav_cli goal ...` | T5：走廊绕障到达 ≥ 9/10；T6：桌面悬空物 10/10 不碰 |
| 录制 | `python3 tools/record.py -o logs/run.jsonl.gz` | 话题 JSONL（点云 base64），用于离线复现 |

## 7. 已知限制 / 待实测项

- `rt/cmd_vel_no_limit` 的超时、限幅、锁轮行为未经实测（`guard.zero_hold` 暂定 1.0 s）。
- `kinematics.head_to_livox`、`base_to_agv_z`、`lift_offset/scale`、自身点框、`nav_pose.arm_tuck` 都是估计值，必须按 `docs/CALIBRATION.md` 标定。
- 底盘 HTTP 事件格式（`type` / `timestamp` 字段名）按文档假设，`EventTracker` 兼容 `timestamp` / `time` / `id`；AEB 开关接口的请求体未确认，guard 不主动调用。
- 仿真中房间场景第二个航点离家具最近 0.09 m（整机轮廓），真机需用 T5 / T6 确认膨胀半径（`costmap.common.inflation.radius`）是否要加大。
- 动态行人只通过体素层的射线清除处理，没有专门的预测。
