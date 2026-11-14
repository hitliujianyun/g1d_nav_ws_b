# g1d_ws：G1-D 激光里程计 + 避障导航（不依赖 ROS）

让宇树 G1-D 轮式人形机器人用**头部的 Livox Mid-360S** 做激光里程计和三维避障，并通过 `rt/cmd_vel_no_limit` 控制底盘完成导航。整个系统只依赖宇树 SDK2（CycloneDDS），机器人上不安装 ROS。

- **定位**：FAST-LIO2（去 ROS / PCL 改造）+ 底盘轮速融合，按身体关节（升降、腰部）实时计算雷达位姿。
- **避障**：Nav2 Costmap2D 体素层（能看到桌沿等悬空障碍）+ 底盘单线雷达层 + 膨胀；NavFn 全局规划 + Nav2 MPPI 局部控制。
- **安全**：独立的 `g1d_base_guard` 是底盘指令的唯一写入者，负责限速、看门狗、近距离反射停车、急停和遥控接管。
- **接入 NavGPT**：发相对航点 `(dx, dy, dyaw)` 和图像时刻，`g1d_nav` 自动补偿推理延迟；失败时返回原因和可通行方向。
- **Web 调试台**：浏览器里看点云、地图、代价地图、路径，点选导航、多航点任务、遥控、急停、验收测试、在线调参、导出地图。

```
Mid-360S ─▶ g1d_lio (FAST-LIO2) ─▶ g1d_nav (代价地图 + NavFn + MPPI + 状态机) ─▶ g1d_base_guard ─▶ rt/cmd_vel_no_limit
                                       ▲ rt/nav/goal                     ▲ 底盘雷达 / 急停 / 遥控
                              NavGPT / Web 调试台 / nav_cli ──────────────┘
```

## 文档

| 文档 | 内容 |
|---|---|
| [`docs/PROJECT_OVERVIEW.md`](docs/PROJECT_OVERVIEW.md) | 项目详细介绍：架构、各模块原理、话题与接口、安全设计、开源代码复用、测试结果 |
| [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md) | 移植到实机：环境准备、编译、分阶段上线、**需要设置 / 标定的参数**、参数参考、故障排查 |
| [`docs/CALIBRATION.md`](docs/CALIBRATION.md) | 外参、身体链、自身点框、底盘行为的标定步骤 |
| [`docs/WEB_UI.md`](docs/WEB_UI.md) | Web 调试台使用说明 |
| [`PLAN.md`](PLAN.md) | 开发计划、TODO、进度日志 |
| `../design/G1D_LIO_MPPI_Nav_Plan.md` | 原始设计方案 |

## 快速开始

### 1. 开发机（Windows / Linux，无需硬件）

依赖：g++ ≥ 9（Windows 用 MinGW-w64）、CMake ≥ 3.16、Ninja、Python 3.8+（`pip install numpy pyyaml pytest cyclonedds matplotlib`）。

```bash
git submodule update --init --recursive
# Windows：编译 + 全部测试 + 离线导航演示（结果图在 logs/sim_*.png）
powershell -ExecutionPolicy Bypass -File scripts\build_host.ps1 -Demo
# 无硬件体验 Web 调试台（模拟雷达 / 导航 / 底盘，guard 为真实进程）
python tools/sim_stack.py          # 浏览器打开 http://localhost:8080
```

### 2. 机器人（G1-D PC2：Jetson Orin NX，Ubuntu 20.04）

```powershell
# 开发机：打包（含子模块，LF 换行，保留符号链接）
powershell -ExecutionPolicy Bypass -File scripts\pack_for_robot.ps1
```

```bash
# 机器人 PC2
tar xzf g1d_ws-<commit>.tar.gz && sudo mv g1d_ws /opt/
/opt/g1d_ws/scripts/build_robot.sh                # 编译、安装、自测
cd /opt/g1d_ws && deploy/run_all.sh               # 启动 guard → web → lio（静止 2 s）→ nav
```

首次上机前必须完成接口确认和标定，按 [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md) 分阶段进行。

## 使用

**浏览器**：打开 `http://192.168.123.164:8080` → 「获取控制权」→ 在地图上点选目标（按下定位置、拖动定朝向），或在「任务」页编排多航点任务。空格键 = 急停。

**命令行**：

```bash
python3 tools/nav_cli.py goal 1.5 0.0 --yaw 0.0 --wait      # 相对当前车体：前 1.5 m
python3 tools/nav_cli.py goal 3.0 1.0 --frame odom --wait   # 里程计坐标系绝对目标
python3 tools/nav_cli.py rotate 1.57 --wait                 # 原地转
python3 tools/nav_cli.py stop                               # 取消
python3 tools/nav_cli.py estop on|off                       # 急停 / 解除
python3 tools/nav_cli.py watch                              # 实时状态
```

**NavGPT 接入**：

```python
from g1d_nav_py.nav_client import NavClient
from g1d_nav_py.navgpt_bridge import NavGptBridge

bridge = NavGptBridge(NavClient(0, "eth0"), camera_latency_s=0.08, max_speed=0.35)
result = bridge.step(lambda: navgpt3.infer(image, instruction), t_recv=frame_recv_time)  # 返回 (dx, dy, dyaw)
prompt += bridge.feedback(result)   # 失败时附带原因和可通行方向，供下一轮推理
```

## 目录

| 路径 | 内容 |
|---|---|
| `src/` | C++：`common` `lio` `nav`（算法核心）、`apps`（`g1d_lio` / `g1d_nav` 可执行程序）、`sim` `tools`、`ros_shim` `pcl_lite`（免 ROS / PCL 垫片） |
| `python/` | `g1d_nav_py`（guard、NavClient、NavGPT 桥、mock 底盘、fake_nav）、`g1d_web`（Web 调试台）、`tests` |
| `tools/` | 接口探测、底盘行为实测、录制、外参标定、命令行、本机仿真栈 |
| `config/` | `g1d_nav.yaml`（所有进程共用参数）、`mid360s_config.json` |
| `deploy/` `scripts/` | systemd 单元、启动脚本、CycloneDDS 配置；构建与打包脚本 |
| `third_party/` | 修改过的上游代码（FAST-LIO2、Nav2 子集）与子模块（Eigen、yaml-cpp、Livox-SDK2、unitree_sdk2 等） |
| `tests/` | C++ 单元测试与闭环仿真测试 |
