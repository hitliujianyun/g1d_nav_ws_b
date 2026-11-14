# g1d_web：调试 / 可视化 / 控制 Web 前端 — 设计

日期：2026-11-14。需求确认：后端跑在机器人 PC2；离线（Python 标准库 + numpy，前端库随仓库）；笔记本浏览器；
建图只做"本次运行的可视化 + 导出"（不做重定位）；控制功能：点选目标、多航点任务、Web 遥控、急停、T1–T6 验收向导、在线参数。

## 1. 进程与数据流

```
g1d_lio ─ registered_cloud, lio_health ┐
g1d_nav ─ status, debug, base_odom, obstacle_points, local/global_costmap, param_state ┤
guard   ─ guard_state, chassis_scan, param_state ┤──DDS──▶ g1d_web (asyncio) ──WebSocket──▶ 浏览器 (three.js)
slamware odom ┘                                  │◀── goal, estop, teleop_cmd, param_set ◀── 用户操作
```

- `g1d_web` 只通过 DDS 与其他进程交互，不直接写 `rt/cmd_vel_no_limit`。Web 遥控发 `rt/nav/teleop_cmd`，
  由 guard 以 deadman（0.3 s 超时）方式接管，仍经过限幅和反射层，并在 `guard_state.manual` 中报告，`g1d_nav` 因此取消当前目标。
- 新增话题：`rt/nav/debug`（nav，JSON，5 Hz：`T_odom_lio`、目标、全局 / 局部路径、MPPI 最优轨迹、限速）、
  `rt/nav/global_costmap`（1 Hz）、`rt/nav/param_set` / `rt/nav/param_state`（`{"target": "guard"|"nav", "name", "value", "id"}`）、
  `rt/nav/teleop_cmd`（Twist_）。

## 2. 后端模块（`python/g1d_web/`）

| 模块 | 职责 |
|---|---|
| `wsproto` | RFC 6455 握手与帧（分片、ping/pong、close），无第三方依赖 |
| `server` | asyncio HTTP：静态文件、`/api/*` JSON、`/files/<maps|tests>/...` 下载、`/ws` 升级 |
| `hub` | DDS 订阅 → 线程安全的最新状态 + 话题频率；发布 goal / estop / teleop / param_set |
| `mapping` | 体素累积 LIO 点云（`T_odom_lio` 变换到 odom）、轨迹；导出 PCD、PNG + YAML（map_server 格式）、CSV、meta.json |
| `mission` | 多航点任务（顺序执行、暂停 / 继续 / 取消、失败策略、驻留时间、循环），保存为 `missions/*.json` |
| `acceptance` | T1–T6 向导与自动计量，结果写 `logs/tests/*.json` |
| `params` | 参数汇总、转发、写回 YAML（保留注释，自动备份） |
| `app` | 客户端管理、控制锁（单控制者，其余只读；急停任何人可按）、消息分发、周期推送 |

## 3. WebSocket 协议

文本帧（JSON，`type` 字段）：

- 服务端 → 客户端：`hello`（client_id、配置：footprint、旋转半径、体素等）、`state`（10 Hz：status、guard、lio、pose、debug、rates、control、mission、test、map 统计）、`params`、`event`（日志）、`exports` / `missions` / `tests`（列表）。
- 客户端 → 服务端：`control {action: acquire|release, force}`、`goal {goal}`、`stop`、`estop {on}`、`teleop {v, w}`（10 Hz）、
  `mission {action: start|pause|resume|cancel|save|delete|list, mission, name}`、`map {action: reset|export|set_voxel|list, voxel}`、
  `param {target, name, value}`、`param_save {target}`、`test {action: start|mark|stop|abort, test, args}`。
  除 `estop on` 外，有副作用的命令都需要控制权。

二进制帧：`[u8 kind][3 字节保留][u32 小端 header 长度][header JSON][补齐到 4 字节][payload]`。

| kind | 内容 | payload |
|---|---|---|
| 1 | 当前帧点云（odom） | float32 xyz |
| 2 | 地图增量（新体素） | float32 xyz |
| 3 | 地图重置 | — |
| 4 | Mid-360S 障碍点（odom） | float32 xyz |
| 5 | 底盘雷达点（odom） | float32 xyz |
| 6 / 7 | 局部 / 全局代价地图 | uint8（header：width、height、resolution、origin） |

## 4. 前端（`python/g1d_web/static/`，原生 ES 模块 + three.js r160）

- 顶栏常驻：连接、导航状态、LIO、guard 拦截原因、控制权、**急停**、**取消导航**。
- 左侧 3D / 俯视视图，图层可开关：地图（按高度着色）、当前帧、障碍点、底盘雷达、局部 / 全局代价地图、全局路径、局部路径、MPPI 轨迹、历史轨迹、机器人、目标、任务航点、可通行扇区。
- 右侧标签页：导航、任务、遥控（虚拟摇杆 + WASD）、建图、测试、参数、诊断。
- 地图点选：按下定位置、拖动定朝向、松开发送（导航页）或添加航点（任务页）。

## 5. 验收测试计量

| 测试 | 操作 | 自动计量 | 通过 |
|---|---|---|---|
| T1 | 对准障碍物 → 开始：Web 以 0.3 m/s 遥控前进直到 guard 拦停 | 停车后 `guard_state.front_clear`（底盘雷达 + 障碍点的扫掠净距） | 每次 ≥ 0.40 m |
| T2 | 开始：0.3 m/s 行驶 2 s 后停止发送指令 | 最后一条指令到轮速 < 0.02 m/s 的时间 | ≤ 0.6 s |
| T3 | 在地面标记起点 → 开始 → 遥控 / 导航绕 10 m 矩形回到标记 → 结束 | 起终点位姿差（`base_odom`） | < 0.10 m 且 < 2° |
| T4 | 开始：自动 12 次 +90° 旋转（3 圈） | 起终点平移漂移 | < 0.05 m |
| T5 / T6 | 开始后每个导航目标（或任务航点）计为一次试验 | 成功率、急停 / 碰撞事件、最小净距 | T5 ≥ 9/10，T6 10/10 |

## 6. 本机验证

- `g1d_nav_py.fake_nav`：模拟 `g1d_lio + g1d_nav` 的全部输出（3D 射线点云、代价地图、A* 路径、跟踪控制），配合 mock 底盘与真实 guard 在本机跑通 Web 全链路。
- pytest：wsproto、地图累积与导出、YAML 写回、任务、验收计量、DDS 端到端（Web 客户端发目标 → 机器人到达）。
- 浏览器冒烟：打开页面、点选目标、运行任务、截图检查。
