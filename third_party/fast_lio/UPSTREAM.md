# Vendored: FAST-LIO2

| 项 | 值 |
|---|---|
| 上游 | https://github.com/hku-mars/FAST_LIO （master） |
| 提交 | `7cc4175de6f8ba2edf34bab02a42195b141027e9`（2024-07-23） |
| ikd-Tree | https://github.com/hku-mars/ikd-Tree @ `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |
| 许可证 | GPL-2.0（见 `LICENSE`）；ikd-Tree 同为 GPL-2.0 |
| 参考 | 去 ROS 思路参考 https://github.com/BurhanMuhyiddin/FAST-LIO-NON-ROS |

本目录最初一次提交是上游原样文件（只保留算法相关子集）。之后对它的每一处修改都在本仓库 git 历史中，
用 `git log -p -- third_party/fast_lio` 查看。

主要改动方向：
- ROS / PCL 依赖替换为 `pcl_lite`（`../../src/pcl_lite`）与本项目的数据结构；
- `laserMapping.cpp` 中的 ROS 节点改造为可嵌入的 `fast_lio_api` 接口（`include/fast_lio_api.h`，
  上层封装见 `../../src/lio/lio_estimator.*`）；
- 动态点（M8）：EKF 更新之后、`map_incremental()` 之前调用可选的逐点动态标注回调
  （`set_point_labeler`）。被标为动态的点及其邻域（`exclusion_voxel`，膨胀一格）不插入 ikd-Tree；
  有回调时插入延迟 `map_delay_scans` 帧，窗口内任一帧在附近标出动态点就丢弃（时间一致性）。
  另加 `delete_boxes()`（按世界坐标盒删除地图点，用于迟到的相机人员证据）与 `map_points()`
  （地图快照）。估计器本身（残差、迭代 EKF、ikd-Tree）未改。
