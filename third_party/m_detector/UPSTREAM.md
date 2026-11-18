# Vendored: M-detector

| 项 | 值 |
|---|---|
| 上游 | https://github.com/hku-mars/M-detector （main） |
| 提交 | `0645dabe307a99f3b59a5a3d3007acfdcad1a39f` |
| 论文 | H. Wu, Y. Li, W. Xu, F. Kong, F. Zhang. *Moving event detection from LiDAR point streams.* Nature Communications 15, 345 (2024) |
| 许可证 | GPL-2.0（见 `LICENSE`），与 FAST-LIO 相同，单独编译为 `g1d_mdetector` 静态库 |

本目录第一次提交是上游原样文件（`DynObjFilter.h/.cpp`、`types.h`、`parallel_q.h`、
`config/mid360/mid360.yaml`、`LICENSE`、`README.md`），之后的改动见
`git log -p -- third_party/m_detector`。

改动（只动基础设施，Case 1/2/3 判据原样保留）：
- 去 ROS：`init(ros::NodeHandle&)` → `init(const m_detector::ParamSource&)`（`compat.h`，参数名保持
  上游的 `dyn_obj/...`）；删除 `publish_dyn()`、`set_path()` 和逐点标签文件输出；
- 去 OpenCV / PCL I/O / `pcl_conversions`；删除依赖 PCL 完整版的 DBSCAN 聚类阶段
  （`DynObjCluster`，`cluster_coupled/cluster_future` 不再生效）。替代它的体素投票细化在
  `../../src/lio/src/dynamic_detector.cpp`；
- `std::execution::par`（依赖 TBB）→ OpenMP `parallel_for`；`boost::shared_ptr` → `std::shared_ptr`；
- `parallel_q.h` 补 `<cassert>/<cstdio>`，`is_empty` 初始化；`DynObjFilter` 析构释放点池；
- `DepthMap()` 去掉调试打印。

参数默认值在 `DynamicDetectorParams::defaults()`：取自上游 `mid360.yaml`，改了 10 Hz 帧、4 张深度图、
关闭聚类和调试输出，以及 case-1 地图一致性窗口 `map_cons_hor_dis1/ver_dis1` 0.3/0.6 → 0.15/0.15
（倒装头部雷达下，人脚边的地面落入大窗口，腿部被判"与地图一致"，静态标签再沿身体向上传播；
仿真召回 0.2 → 0.7，误检仍为 0）。保留上游怪癖：键 `ver_resolution_max` 设置的是水平分辨率，反之亦然。
