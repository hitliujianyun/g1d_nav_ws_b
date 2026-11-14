# 标定与参数实测

所有结果写回 `config/g1d_nav.yaml`。机器人放在平整地面上，周围 2 m 内无遮挡。

## 1. 身体链（`kinematics.*`）

`g1d_nav` 用 URDF 链计算 `base_link → head_link → livox → livox IMU`：

```
base_link ─(base_to_agv_z)→ AGV_link ─LZ_ot 0.5275─→ 升降两级（各 h/2，总行程 ≤ 0.42 m）
          → Pitching → Yaw_Joint（腰俯仰，绕 y）→ torso（腰偏航，绕 z）→ head_link ─(head_to_livox)→ livox
```

1. 升降高度：`python3 tools/check_interfaces.py`，读 `rt/hispeed_state` 的 `height_y`。用卷尺量出升降实际行程后，求 `lift_scale`、`lift_offset`，使 `lift_scale · y + lift_offset` 等于实际总行程。
2. 打印当前配置下的链：
   ```bash
   install/bin/g1d_fk config/g1d_nav.yaml <height_y> <waist_yaw> <waist_pitch>
   ```
   输出 `base->livox xyz / rpy`，下一步要用。

## 2. Mid-360S 外参（`kinematics.head_to_livox`）

1. 在 `g1d_nav.yaml` 设 `debug.publish_lidar_cloud: true`，用 `deploy/run_all.sh` 启动（需要 guard 提供 `rt/nav/chassis_scan`，`g1d_lio` 提供 `rt/nav/livox_cloud`）。
2. 腰部回零（`waist_yaw = waist_pitch = 0`，此时 `base->head` 旋转为单位阵）：
   ```bash
   python3 tools/calib_floor.py --fk-xyz <x y z> --fk-rpy <r p y> --yaw-from-scan
   ```
   - 地面 RANSAC → 雷达横滚 / 俯仰和离地高度；与 FK 高度的差值说明 `head_to_livox.z`（或 `base_to_agv_z`）的误差。
   - `--yaw-from-scan`：把雷达点云在底盘雷达高度切一层，与 `rt/nav/chassis_scan` 做偏航搜索（±10°，0.1° 步长）。
3. 按输出更新 `head_to_livox.rpy`（腰部为零时可以直接采用）和 `z`，重跑 2，直到倾斜误差 < 0.3°、高度误差 < 1 cm、偏航修正 < 0.3°。
4. 验证：原地转 3 圈（T4），看 `rt/nav/base_odom` 的平移漂移；外参偏差会表现为绕圈。

`lio.extrinsic_T/R`（雷达与内置 IMU）采用 FAST-LIO `mid360.yaml` 的出厂值，一般不改。

## 3. 自身点过滤框（`self_filter.boxes`）

`self_filter.boxes` 是 head_link 系下的盒子 `[xmin, xmax, ymin, ymax, zmin, zmax]`，盒内点（头、躯干、立柱、收拢的手臂）在进入 FAST-LIO 和代价地图前删除。

1. 摆好导航姿态（手臂收拢、腰部回零），录 5 s：`python3 tools/record.py --topics obstacles -o logs/self.jsonl.gz --duration 5`。
2. 如果 `rt/nav/obstacle_points` 里在车身轮廓（`guard.body_x/body_y`）以内或紧贴车身有固定点，说明盒子偏小：放大对应的盒子，直到静止时车身周围 0.3 m 内没有障碍点。

## 4. 导航姿态锁定（`kinematics.nav_pose`）

摆好导航姿态后运行 `check_interfaces.py`，把 `rt/lowstate` 样本里的 `arms_q15_28`（14 个值）填入 `kinematics.nav_pose.arm_tuck`，`rt/waistpitchstate` 的 `q` 填入 `nav_pose.waist_pitch`。导航中姿态偏离 `lock_tolerance` 时 `g1d_nav` 以 `body_moved` 失败。

## 5. 底盘指令行为（`guard.*`）

停掉 guard 后运行 `python3 tools/cmd_vel_probe.py --i-know-it-moves --tests silence,zero,clamp,hold`：

| 实测项 | 写回 |
|---|---|
| `silence.stop_time_s`：停止发送后到停车的时间 | `guard.zero_hold` ≥ 该值 + 0.3 s（工具给出建议值） |
| `zero.stop_distance_m`：发零速时的刹车距离 | 核对 `guard.brake_v` 和 `stop_m`（停车距离 = 反射层触发距离 − 刹车距离 ≥ 0.40 m） |
| `clamp.v_reached / w_reached` | 若底盘自身截断，`mppi.vx_max` / `wz_max` 不要超过该值 |
| `hold.locked_while_zero` | 若发零速会锁轮，`zero_hold` 尽量短，以免影响推车和遥控 |

## 6. 时钟

`rt/nav/lio_health` 里的 `clock_offset`（雷达时钟 − 主机时钟）应保持稳定，`clock_jumps` 应为 0。持续漂移说明 Mid-360S 没有时间同步，FAST-LIO 仍可工作（`ClockMapper` 做最小偏移映射），但 NavGPT 的 `base_at_stamp` 延迟补偿精度会下降。
