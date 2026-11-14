# g1d_web 调试 / 可视化 / 控制前端

浏览器打开 `http://<PC2 IP>:8080`（Chrome / Edge）。后端 `g1d_web` 跑在机器人 PC2 上，只依赖 Python 标准库、numpy、PyYAML 和 unitree_sdk2py，前端库 three.js r160 随仓库提供，离线可用。设计见 [`superpowers/specs/2026-11-14-web-debug-ui-design.md`](superpowers/specs/2026-11-14-web-debug-ui-design.md)。

## 1. 启动

| 场景 | 命令 |
|---|---|
| 机器人（手动） | `deploy/run_all.sh`（guard → web → lio → nav），或单独 `python3 -m g1d_web` |
| 机器人（systemd） | `sudo deploy/install.sh --enable`，`sudo systemctl start g1d-web` |
| 本机无硬件演示 | `python tools/sim_stack.py`，打开 `http://localhost:8080` |

`tools/sim_stack.py` 同时启动 `g1d_nav_py.fake_nav`（模拟 g1d_lio + g1d_nav 的全部输出：Mid-360S 射线点云、代价地图、A* 路径、跟踪控制；内含 mock 底盘）、**真实的** `g1d_base_guard` 和 `g1d_web`。场景 `lab`：房间内有箱子、桌子（桌面悬空）、柱子、隔断和柜子。

## 2. 页面

- **顶栏（常驻）**：连接状态、导航状态 / 原因、LIO 健康、guard 状态（拦截原因、急停、人工接管），实时位姿；控制权按钮、**取消导航**、**急停**（空格键）、解除急停。
- **3D 视图**：左键旋转、右键平移、滚轮缩放；「俯视 2D」切换为俯视；「跟随」让视角跟着机器人。左上角图层开关：累积地图（按高度着色）、当前帧点云、障碍点、底盘雷达、局部 / 全局代价地图、全局路径、局部路径、MPPI 轨迹、历史轨迹、可通行扇区、任务航点、网格。
- 左下角 HUD：nav 输出指令、guard 实际下发给底盘的指令及来源（nav / teleop）、前方净距、地图点数。

## 3. 控制权

同一时间只有一个浏览器能控制机器人（「获取控制权」），其他页面只读，可以「强制接管」。控制者断开后控制权自动释放。**急停任何页面都能按**；解除急停、下发目标、遥控、任务、参数、测试需要控制权。

## 4. 各标签页

| 标签 | 功能 |
|---|---|
| 导航 | 模式 navigate / simple / rotate，坐标系 odom（地图坐标）或 base（相对当前车体），限速、容差；「地图点选」：按下定位置、拖动定朝向、松开发送；快捷：前进 1 m、左 / 右转 90°、掉头；状态面板（剩余距离、朝向误差、恢复次数、LIO / 姿态锁定、可通行扇区条） |
| 任务 | 航点表（x、y、朝向、模式、停留时间，可上移 / 删除），「地图添加航点」连续点选，「添加当前位置」；循环次数（0 = 无限）、失败策略（停止 / 跳过 / 重试）；开始 / 暂停 / 继续 / 取消；保存到 `missions/<名称>.json`、载入、删除；执行结果表 |
| 遥控 | 虚拟摇杆（按住才动）或键盘 W/A/S/D；最大线 / 角速度滑块。经 `rt/nav/teleop_cmd` 交给 guard：0.3 s 收不到即停（deadman），优先于导航并取消导航目标，仍受限速和反射层约束 |
| 建图 | 本次运行的累积点云（`T_odom_lio` 变换到 odom）：点数、帧数、体素、对齐状态；重置（可改体素）、暂停累积、着色方式；**导出**到 `maps/<时间>/`：`map.pcd`、`grid.png + grid.yaml`（map_server 格式，占据 = 0.10–1.80 m 高度带内的点）、`trajectory.csv`、`costmap.png + costmap.yaml`、`meta.json`，页面直接下载 |
| 测试 | T1–T6 验收向导：T1 自动以设定速度驶向障碍物并记录停车净距；T2 行驶后中断指令测停车时间；T3 起点 → 绕行 → 标记终点算闭环误差；T4 自动 12 × 90° 旋转测平移漂移；T5 / T6 记录期间每个导航目标计一次试验（成功率、急停、最小净距）。结果写 `logs/tests/*.json` 并可下载 |
| 参数 | guard 与 nav 的可热更新参数（guard：限速、加速度、停车 / 减速距离、爬行速度、旋转半径、零速保持、遥控限速等；nav：默认限速 `max_speed`、末段接近速度、旋转增益、进度窗口、恢复参数）。「应用」立即生效；「保存」写回 `config/g1d_nav.yaml`（保留注释，自动备份 `.bak-<时间>`） |
| 诊断 | 各话题频率和距上次收到的时间、事件日志、nav status / guard_state / lio_health / debug / 指令的原始 JSON |

## 5. 话题（新增）

| 话题 | 方向 | 说明 |
|---|---|---|
| `rt/nav/debug` | g1d_nav → web | JSON 5 Hz：`T_odom_lio`、目标、全局 / 局部路径、MPPI 轨迹、限速 |
| `rt/nav/global_costmap` | g1d_nav → web | OccupancyGrid_ 1 Hz（`debug.publish_costmap`） |
| `rt/nav/param_set` / `rt/nav/param_state` | web ↔ guard / nav | `{"target", "name", "value", "id"}` / `{"target", "params", "ack"}` |
| `rt/nav/teleop_cmd` | web → guard | Twist_，deadman `guard.teleop_timeout` |

## 6. 注意

- Web 不直接写 `rt/cmd_vel_no_limit`，guard 仍是唯一写入者；遥控同样经过反射层，靠近障碍会被拦停。
- 地图只在 `g1d_nav` 发布 `T_odom_lio` 后累积（否则建图页显示"等待 T_odom_lio"）。LIO 重启后 odom 原点改变，应「重置地图」。
- 页面不做身份认证，只应在机器人内网（192.168.123.x）使用。
