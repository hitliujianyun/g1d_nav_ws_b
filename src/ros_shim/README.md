# ros_shim

最小 ROS 2 API 垫片，只为让 `third_party/nav2` 中的 Nav2 算法代码（costmap_2d 核心、voxel_grid、navfn、
mppi_controller）在**不安装 ROS** 的情况下原样编译运行。

- 只实现被 vendored 代码实际用到的符号（见 `../../PLAN.md` 第 4 节与 git 历史）。
- 消息类型是字段名与 ROS 消息一致的纯结构体；不涉及序列化，也不与 DDS 通信。
- 参数系统是进程内键值表（`nav2::LifecycleNode`），由 `g1d` 代码从 YAML 填充。
- `pluginlib` 换成静态注册表：`PLUGINLIB_EXPORT_CLASS` 在静态初始化时注册工厂函数，
  因此相关目标文件必须整体链接（CMake 中使用 `WHOLE_ARCHIVE`）。
- 日志宏输出到 `rclcpp::detail::log_handler()`，默认写 stderr，级别由环境变量 `G1D_LOG_LEVEL`
  （DEBUG / INFO / WARN / ERROR）控制。
