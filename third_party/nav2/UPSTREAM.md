# Vendored: Nav2 子集

| 项 | 值 |
|---|---|
| 上游 | https://github.com/ros-navigation/navigation2 （main） |
| 提交 | `4c015756151f303012a42505ef54c41d5078ec5c`（2026-11-12） |
| 许可证 | 见各包 `package.xml`：nav2_costmap_2d（BSD-3-Clause, Apache-2.0）、nav2_voxel_grid（BSD-3-Clause）、nav2_navfn_planner（Apache-2.0, BSD-3-Clause）、nav2_mppi_controller（MIT）、nav2_util（Apache-2.0, BSD-3-Clause）、nav2_core（Apache-2.0）；仓库总 `LICENSE` |

只保留以下算法相关文件：

| 包 | 内容 |
|---|---|
| nav2_costmap_2d | `Costmap2D`、cost_values、footprint、FootprintCollisionChecker、costmap_math、exceptions、filter_values；`inflation_layer.*` 仅作移植参考（不编译） |
| nav2_voxel_grid | `VoxelGrid` |
| nav2_navfn_planner | `NavFn` 核心（不含 ROS 插件 `navfn_planner.cpp`） |
| nav2_mppi_controller | 全部 include / src（`controller.cpp`、`trajectory_visualizer.cpp` 不编译） |
| nav2_util / nav2_core | 上述代码引用的工具头文件与接口 |

本目录最初一次提交是上游原样文件；之后的修改见 `git log -p -- third_party/nav2`。
ROS 依赖通过 `../../src/ros_shim`（最小 ROS API 垫片）满足，算法代码尽量不改动。
