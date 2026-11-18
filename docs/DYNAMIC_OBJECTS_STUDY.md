# 动态物体过滤：调研与方案选型

日期：2026-11-18。目标有两个：

1. **点云地图**（FAST-LIO2 的 ikd-Tree 地图、Web 导出的 `map.pcd` / `grid.png`）不留下行人等动态物体的"拖影"；
2. **导航代价地图**不被已经离开的动态物体干扰：人走过后，地图能及时恢复为空闲。

约束：机器人 PC2 为 Jetson Orin NX（Ubuntu 20.04 / JetPack 5），当前系统不依赖 ROS，CPU 已被 LIO + MPPI 占用一部分，GPU 目前空闲；头部有 Mid-360S（非重复扫描，倒装）和相机。

---

## 1. 现状与问题来源

| 位置 | 现在的做法 | 动态物体的影响 |
|---|---|---|
| FAST-LIO2 ikd-Tree | 每帧配准后，降采样点全部加入地图（`map_incremental()`），不判断动态 | 行人轨迹留在 ikd-Tree 中，直到移出约 60 m 范围；人多时可能拉偏配准 |
| `g1d_nav` 体素层 | 当前帧点云：先沿射线清除，再标记；`decay_s = 0` | 只有射线穿过才清除；没有回波的方向、雷达盲区和视场外的拖影会残留 |
| 底盘雷达层 | 逐帧标记与二维清除，`inf_is_valid: false` | 没有回波的方向不清除 |
| guard 反射层 | 只用当前帧 | 不受影响 |
| Web 累积地图 / 导出 | 体素只增不删 | 拖影全部进入导出的 `map.pcd` 和 `grid.png` |

两个问题本质不同，应分开解决：**地图**要求"动态点永不入库"（宁可多删，事后可修），**代价地图**要求"刚看到的要避开、离开的要尽快忘掉，但盲区里的不能忘"。

## 2. 方法调研

### 2.1 传统几何方法

| 方法 | 原理 | 在线 / 离线 | 性能（论文数据） | 依赖 / 许可证 | 与本系统的适配 |
|---|---|---|---|---|---|
| **M-detector**（HKU MARS，Nature Communications 2024） | 遮挡原理：当前点与历史深度图的遮挡关系不一致 → 运动点；逐点判断 | 在线 | 每点延迟 2–4 μs；Livox AVIA 室内数据集每帧 16.1 ms（帧周期 100 ms），桌面 i7-10700 [1] | ROS1、PCL、TBB；**GPL-2.0**（与 FAST-LIO 相同）[2] | 官方就是配合 FAST-LIO 和 Livox 使用，输入为本体系点云 + 里程计 [2][3]；默认视场参数 `fov_up 52 / fov_down -7` 正是 Mid-360 [2] |
| **DUFOMap**（KTH，RA-L 2024） | 射线投射标记"曾经被观测为空"的 void 体素，落入 void 的点即动态 | 在线（DUFOMap*）和离线 | 每帧 0.019 s（16 线）/ 0.062 s（64 线），桌面 i9-12900KF；各数据集用同一组参数（体素 0.1 m） [4] | UFOMap、TBB、gcc-10；**BSD-3**；有 `pip install dufomap`（Python ≥ 3.8）[5] | 支持非重复扫描、演示过 Livox Mid-360 数据集 [4][5]；离线清理精度在基准中领先 |
| **Dynablox**（ETH，RA-L 2023） | TSDF + 高置信空闲空间，进入空闲空间的物体判为动态 | 在线 | 17 FPS（AMD-4800U 笔记本 CPU，20 m 范围 58 ms/帧）[6] | voxblox、ROS1 | 已被 NVIDIA nvblox 集成到 GPU 实现 [6]；稀疏雷达上动态召回较低 [7] |
| **BeautyMap / ERASOR / Removert** | 与先验地图比较（离线） | 离线 | BeautyMap 在 KITTI 与半室内数据上综合最好；ERASOR 删得多但误删静态；Removert 删得少 [7][8] | 需要先有完整地图 | 只适合地图建完后的清理 |
| **OctoMap 射线清除** | 占据概率 + 射线投射 | 两者 | 慢（每帧 1–3 s），地面角度问题导致静态精度低 [4][8] | — | 不推荐 |
| **STVL**（Nav2 时空体素层） | 体素按时间衰减；处于传感器视场锥内但没再被看到的体素加速衰减（不做射线） | 代价地图 | 资源占用比 VoxelLayer 低约 2 倍 [9] | Nav2 插件 | 思路可直接借鉴到本项目的体素层；**视场锥内被车身遮挡的区域需要排除**，否则会误清除 [10] |

KTH 动态地图基准的关键数据（点级精度：SA 为静态点保留率、DA 为动态点剔除率）[7]：

| 方法 | KITTI 00 SA / DA | 半室内（16 线，两人走动）SA / DA |
|---|---|---|
| DUFOMap | 97.96 / 98.72 | 99.64 / 83.00 |
| Dynablox | 96.76 / 90.68 | 98.81 / 36.49 |
| ERASOR | 66.70 / 98.54 | 94.90 / 66.26 |
| Removert | 99.44 / 41.53 | 99.96 / 12.15 |
| OctoMap | 68.05 / 99.69 | 88.97 / 82.18 |

结论：几何方法里，**在线逐点检测选 M-detector，地图清理选 DUFOMap**。这两者都开源、都在 Livox 数据上演示过，也都不需要训练数据。

### 2.2 结合 FAST-LIO2 的现成改版

| 项目 | 做法 | 评价 |
|---|---|---|
| `better_fastlio2`（Yixin-F） | FAST-LIO2 + 曲面体素占据描述子动态剔除（T-GRS 2024）+ 可选 YOLO，后端 SC + GTSAM [11] | 功能多、依赖重（GTSAM、ROS1），其动态模块原文为离线方法，作者在线化时牺牲了精度 [11] |
| `FAST-LIO-WithoutDynObj`（LeeWhisper） | 基于光路原理在 FAST-LIO 中剔除动态点，Mid-360 隧道场景测试 [12] | 作者自述误识别率较高、只对近距离有效、结果不连续 [12] |
| Dynamic-LIO（IROS 2025） | 地面 / 非地面标签一致性，每帧 1–9 ms [13] | 前端是 SR-LIO，不是 FAST-LIO2；针对驾驶场景 [13] |
| RF-LIO | 先剔除再配准，基于 LIO-SAM [14] | 架构不同，不便移植 |

结论：没有可以直接替换、又足够成熟的"FAST-LIO2 + 动态剔除"版本。更稳妥的做法是在现有 FAST-LIO2 外面接一个独立的检测模块（M-detector），只改"哪些点入库"。

### 2.3 学习方法（小模型）

| 方法 | 情况 | 是否适合本系统 |
|---|---|---|
| 4DMOS / MapMOS（激光雷达运动分割） | 依赖 MinkowskiEngine 稀疏卷积；4DMOS 在 RTX A5000 上 5 帧 47 ms、10 帧 78 ms [15][16] | 不推荐：Jetson 上部署困难；在 16 线半室内数据上 DA 只有 10.6% / 4.75%，对稀疏、非 KITTI 类雷达泛化差 [7] |
| LiMoSeg（BEV 运动分割） | Jetson Xavier 上 8 ms [17] | 需要针对 Mid-360S 和室内场景重新训练，没有现成权重可用 |
| **相机分割 + 投影到点云**（YOLO-seg 等） | 对图像做人体分割，把激光点投影到图像，落在人体掩码内的点标为动态；可导出 ONNX / TensorRT 在 Orin NX 上运行 [18][19]；需加几何校验避免误删背景 [20] | **可选增强**：能识别**静止站立**的人（几何方法只能检测运动中的物体）；但只覆盖相机前方视场，需要相机-雷达外参标定 |
| NVIDIA nvblox | 动态模式基于 Dynablox，人体分割模式用 DNN；占据概率随时间衰减到 0.5，离开视野的物体逐渐变为未知 [21][22]；2026-01 的 v0.0.9 起支持激光雷达输入和动态检测 [23] | 适合 **ROS 2 版本**：Isaac ROS nvblox 需要 ROS 2 Humble 与 JetPack 6.1 / 6.2 [24]；C++ 库支持 JetPack 5（已标记为弃用）[25] |

结论：激光雷达学习模型目前不划算。小模型最有价值的用法是**相机人体分割**，用来补几何方法"检测不到静止的人"这一短板，并作为可选增强。

## 3. 方案选型

### 3.1 总体结构（推荐）

```
Mid-360S ─▶ FAST-LIO2 (IESKF 配准) ─▶ M-detector（逐点动态标签）──┬─▶ 静态点 ──▶ ikd-Tree（入库）
                │ 位姿                                              │             ▶ registered_cloud（静态）
                └───────────────────────────────────────────────────┴─▶ 动态点 ──▶ rt/nav/dynamic_points
                                                                                     │
            （可选）头部相机 ─▶ 人体分割 TensorRT ─▶ 投影标签 ──────────────────────┘

g1d_nav 代价地图：静态体素层（射线清除 + 视场锥内加速衰减，盲区不衰减）
                  + 动态层（只放 dynamic_points，生存期 0.5–1 s）+ 底盘雷达层 + 膨胀
Web 地图导出：关键帧（点云 + 位姿）─▶ DUFOMap 离线清理 ─▶ map.pcd / grid.png
```

### 3.2 各问题的选型与理由

| 问题 | 选型 | 理由 | 备选 |
|---|---|---|---|
| 动态点进入 ikd-Tree | **M-detector** 接在 FAST-LIO2 的状态更新之后、`map_incremental()` 之前，动态点不入库 | 专为 Livox + FAST-LIO 设计、逐点低延迟、GPL-2 与 FAST-LIO 相同；只改"入库"一处，不动 IESKF | DUFOMap* 在线模式（BSD-3，但需 UFOMap + TBB，计算量更大） |
| 代价地图残留拖影 | 动态点单独进入**短生存期的动态层**；静态体素层加 **STVL 式视场锥内加速衰减**（盲区、车身遮挡区、`< 0.6 m` 不衰减） | 动态物体"在的时候能避、走了立刻忘"；静态层也能清掉射线照不到的残留，同时不丢盲区里的障碍 | 只设全局较短 `decay_s`（实现最简单，但会遗忘真实的静态障碍） |
| 底盘雷达无回波方向不清除 | 实测 SLAMWARE 扫描是否用 `inf` 表示无回波；若是，打开 `inf_is_valid` 按最大量程清除 | 改动一个参数 | — |
| 导出地图的拖影 | **DUFOMap 离线清理**（Web 导出时调用，`pip install dufomap`） | 基准中离线精度最好，参数不用针对场景调整，BSD-3 许可证 | BeautyMap（需先有完整地图） |
| 静止站立的人 | （可选）相机人体分割投影 | 几何方法只检测运动；分割能识别静止的人，避免他走开后留下"人形障碍" | ROS 2 版本可用 nvblox 的人体分割 / 动态模式 |
| LIO 在人多场景的稳健性 | 动态点不参与入库后，后续帧不会与拖影匹配；动态点也可以排除在当前帧残差之外（二期） | 减少错误匹配 | — |

### 3.3 不选的方案及原因

- **4DMOS / MapMOS 等激光雷达学习模型**：Jetson 上部署依赖重，对 Mid-360S 这类非重复扫描雷达缺乏训练数据，基准中在稀疏雷达上表现差 [7][15][16]。
- **直接换成 `better_fastlio2` 等改版**：引入 GTSAM / ROS1 等大量依赖，动态模块的在线精度存在折中，维护成本高 [11]。
- **OctoMap 射线清除**：慢、静态精度低 [4][8]。

## 4. 实施计划

| 阶段 | 内容 | 本机可验证 | 工作量（估计） |
|---|---|---|---|
| **一期：代价地图不残留**（不依赖检测器） | 体素层加视场锥加速衰减（Mid-360S 视场模型、盲区 / 遮挡排除）与分层衰减（局部 / 全局）；底盘雷达 `inf_is_valid` 实测；在 C++ 仿真里加入"行人穿过"场景（`World` 已支持移动盒子） | 是：测"行人离开 → 栅格清空"的时间、盲区障碍是否保留、导航不碰撞 | 2–3 天 |
| **二期：地图不入库** | 移植 M-detector 为无 ROS 库（参照 FAST-LIO 的改法：PCL → `pcl_lite`，TBB → OpenMP / 标准库），接入 `g1d_lio`；动态点不入 ikd-Tree，发布 `rt/nav/dynamic_points`；`g1d_nav` 增加动态层 | 是：用仿真雷达加移动物体测动态点的检出率 / 误检率、ikd-Tree 拖影数量、LIO 误差 | 1–2 周 |
| **二期：导出清理** | Web 导出时保存关键帧（点云 + 位姿），调用 DUFOMap 离线清理，输出清理前 / 后两版地图 | 是（`pip install dufomap` 支持 Windows） | 2–3 天 |
| **三期（可选）：相机分割** | 头部相机 + YOLO-seg（TensorRT），投影到雷达点并做几何校验；与 M-detector 标签取并集 | 部分（需真机相机与外参） | 1–2 周 |
| 真机验证 | Orin NX 上测 M-detector 耗时（目标 < 20 ms/帧）、CPU 余量；行人穿行、门开合、人站立后离开等场景 | — | — |

**验收指标（建议）**：

- 代价地图：行人离开后，局部地图中对应栅格在 ≤ 1 s 内清空（视场内），盲区内的静态障碍保持 ≥ 30 s。
- 地图：人员走动的录制数据上，导出地图的动态点剔除率 DA ≥ 90%、静态保留率 SA ≥ 95%（人工标注少量帧评估）。
- LIO：有行人干扰时，T3 闭环误差与无干扰时相比恶化不超过 20%。
- 资源：动态检测增加的 CPU 占用不超过 1 个核心，LIO 单帧处理时间仍 < 100 ms。

## 5. 风险

- M-detector 依赖准确位姿；**比里程计漂移还慢的运动无法检出**（nvblox 文档对同类方法也有同样说明 [22]）。
- Livox 非重复扫描的单帧覆盖不完整，视场锥加速衰减的时间常数要实测，不能假设"视场内每帧都被完整观测"【推断，依据 STVL 的前提假设 [10]】。
- 对 M-detector 在 Orin NX（ARM）上的耗时没有公开数据，论文只在 x86 桌面和 i7 机载电脑上测过 [1]，必须实测。
- GPL-2.0：M-detector 与 FAST-LIO 同为 GPL-2.0，项目整体已受 GPL-2.0 约束，不引入新的许可证问题；DUFOMap 为 BSD-3。
- 相机分割只覆盖前方视场，且需要相机-雷达外参和时间同步。

## 6. 参考资料

1. Wu et al., *Moving event detection from LiDAR point streams*, Nature Communications 2024 — https://www.nature.com/articles/s41467-023-44554-8
2. hku-mars/M-detector README 与 LICENSE（GPL-2.0）— https://github.com/hku-mars/M-detector
3. M-detector Issue #48（与 FAST-LIO 配合的方式）— https://github.com/hku-mars/M-detector/issues/48
4. Duberg, Zhang et al., *DUFOMap: Efficient Dynamic Awareness Mapping*, RA-L 2024 — https://arxiv.org/abs/2403.01449
5. KTH-RPL/dufomap README 与 LICENSE（BSD-3）— https://github.com/KTH-RPL/dufomap
6. Schmid et al., *Dynablox*, RA-L 2023 — https://arxiv.org/abs/2304.10049 ；https://github.com/ethz-asl/dynablox
7. Jia et al., *BeautyMap*, RA-L 2024（Table I）— https://arxiv.org/abs/2405.07283
8. Zhang et al., *A Dynamic Points Removal Benchmark in Point Cloud Maps*, ITSC 2023 — https://arxiv.org/abs/2307.07260 ；https://github.com/KTH-RPL/DynamicMap_Benchmark
9. Nav2 文档：Using an External Costmap Plugin (STVL) — https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_stvl/
10. STVL PR #357（3D 雷达视场锥的遮挡多边形）— https://github.com/SteveMacenski/spatio_temporal_voxel_layer/pull/357
11. Yixin-F/better_fastlio2 — https://github.com/Yixin-F/better_fastlio2
12. LeeWhisper/FAST-LIO-WithoutDynObj — https://github.com/LeeWhisper/FAST-LIO-WithoutDynObj
13. Yuan et al., *LiDAR-Inertial Odometry in Dynamic Driving Scenarios using Label Consistency Detection* — https://arxiv.org/abs/2407.03590 ；https://github.com/ZikangYuan/dynamic_lio
14. *RF-LIO* — https://arxiv.org/abs/2206.09463
15. Mersch et al., *Receding Moving Object Segmentation ... Sparse 4D Convolutions*（4DMOS）, RA-L 2022 — https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/mersch2022ral.pdf
16. PRBonn/MapMOS — https://github.com/PRBonn/MapMOS
17. *LiMoSeg* — https://arxiv.org/abs/2111.04875
18. LiDAR-Motion-Segmentation/baseline-scripts（YOLO / SAM 投影、Orin NX TensorRT 部署）— https://github.com/LiDAR-Motion-Segmentation/baseline-scripts
19. adityapatel149/rgb-lidar-imu-3d-reconstruction（图像掩码投影剔除动态点）— https://github.com/adityapatel149/rgb-lidar-imu-3d-reconstruction
20. *Geometry-Aware Human Noise Removal from TLS Point Clouds via 2D Segmentation Projection* — https://pmc.ncbi.nlm.nih.gov/articles/PMC12944119/
21. Isaac ROS nvblox 概念页 — https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html
22. nvblox Technical Details（动态重建、占据衰减）— https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html
23. nvblox CHANGELOG v0.0.9（2026-01-27）— https://github.com/nvidia-isaac/nvblox/blob/public/CHANGELOG.md
24. Isaac ROS nvblox 支持平台（release-3.2）— https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html
25. nvblox 安装与支持平台 — https://nvidia-isaac.github.io/nvblox/public/pages/installation.html
