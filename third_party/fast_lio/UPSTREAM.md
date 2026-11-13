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
- `laserMapping.cpp` 中的 ROS 节点改造为可嵌入的 `FastLio` 接口（`../../src/lio/fast_lio_core.*`）。
