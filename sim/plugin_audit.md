# Livox 仿真插件接入核对

## 扫描一致性修正（2026-10-06）

- Classic 插件现在以 1 kHz Gazebo 传感器回调驱动射线；每次只对当前时刻到达的一小批方向求交，并跨 100 ms 累积成 4000 点帧后发布。点的坐标由该次 Gazebo 位姿下的求交得到，PointCloud2 的 `offset_time` 记录相同仿真时刻相对帧起点的纳秒偏移，帧头时间是扫描起点。
- 由于 CSV 首列单位仍未知，当前帧内发射时刻按点序均匀分布在 100 ms 内；**这是显式、可复现的仿真时序假设，不代表 MID-360 硬件真实的逐点发射时序**。方向仍来自 CSV，点数仍为 4000/帧，消息目标频率仍是 10 Hz。
- `plugin_mid360_adapter.py` 对新数据直接转发 `offset_time`；为兼容旧 rosbag，若输入消息没有该字段，才保留旧的均匀时间估算。
- 本次改动已在本机通过 `colcon build --packages-select ros2_livox_simulation --symlink-install` 编译。仿真运行时的消息频率、4000 点完整帧及运动状态下逐点时间与真值的一致性，仍需启动场景采集确认。

以下旧采集记录描述的是修正前的单时刻整帧求交，不应再用来推断当前插件的帧内采样方式。

当前工作区的 `FAST_LIO` 包实际以 FAST-LIO2 模式运行：`sim/fastlio_sim.yaml` 设置 `feature_extract_enable: false`，源码使用 `ikd-Tree` 增量建图。包名 `fast_lio` 不能用来判断算法是第一版。插件接入时仍需提供有效的逐点时间和 IMU，以支持运动补偿。

## 已下载与构建

- 首选：`src/ros2_livox_simulation`，来源 `LCAS/livox_laser_simulation_ros2`，提交 `dbac363e867676efbe7394df32fc3a90f4dfa35e`，仓库根目录有 MIT `LICENSE`。
- `colcon build --symlink-install --packages-select ros2_livox_simulation` 在本机 ROS 2 Humble + Gazebo Classic 11 编译通过。
- 仓库包含 `scan_mode/mid360.csv`、`urdf/mid360.xacro` 和 MID360 外观 DAE。整仓约 239 MB；其中扫描 CSV 占大部分。

## 实际接口与时间语义

- `livox_points_plugin.cpp` 的 `Load()` 创建的是 `sensor_msgs/msg/PointCloud2` 发布器；`OnNewLaserScans()` 只调用 `cloud2_pub->publish(cloud2)`。虽然头文件声明了 `CustomMsg` 发布器，实际没有创建或发布。
- `PointCloud2` 的 `tag`、`line` 固定为 0。当前 FAST-LIO2 配置关闭特征提取，源码仍检查 `line < scan_line` 和 `tag & 0x30`；两个 0 值通过筛选。这里的 `line` 是 Livox 通道编号，不能把占位 0 当作真实通道。若开启特征提取，所有点会被归到同一通道，结果不可靠。
- 已给插件原始 `PointCloud2` 增加 `csv_time_raw`（原样保存 CSV 首列）及 `scan_sequence`，便于独立检查点序与帧序；这两个字段**不是**可直接交给 FAST-LIO2 的纳秒采样时间。
- CSV 原始时间单位没有文档依据，因此不直接把它当作发射时刻；新的插件使用明确的 100 ms 点序时间表并随 Gazebo 更新分批求交。方向序列/均匀时序仍是近似模型，不能声称复现了真实硬件的逐点调制时序。

### CSV 与仿真时间实测

- `mid360.csv` 表头为 `Time/s,Azimuth/deg,Zenith/deg`，共 800000 行；首列从 1 到 800000，全部相邻差值为 1。插件仓库没有给出这列的可靠单位说明，不能按表头直接解释为秒。
- 当前场景的扫描频率是 10 Hz，`samples=4000`、`downsample=1`，相当于每 100 ms 使用 4000 个方向，即 40000 个方向/秒。MID-360 官方标称第一回波 200000 点/秒、典型帧率 10 Hz；当前仿真采样量为标称值的五分之一。这个对比只描述配置差异，不证明 CSV 每行对应硬件一个回波。
- 一次静止 House 采集在 `sim/output/raw_SWZy2gxp/`：插件帧序号 20/21/22 分别包含 CSV 原始值 80001–84000、84001–88000、88001–92000，对应 Gazebo 帧时间 2.1/2.2/2.3 s。顺序连续，但没有证据能从 CSV 首列推出逐点物理采样时刻。
- 索引推进规则来自插件源码：每次 `OnNewLaserScans()` 读取从 `currStartIndex` 开始的 4000 行，之后 `currStartIndex += samplesStep`；读取时对 800000 行取模，因而在当前配置下每 200 帧回到 CSV 开头。`downsample=1`，不跳行。100 帧实测的第 1/10/30/100 帧分别对应 CSV 80001–84000、116001–120000、196001–200000、476001–480000，消息戳为 2.1/3.0/5.0/12.0 s。
- **消息时间窗口**：Gazebo 的 10 Hz 更新使相邻消息戳间隔 0.1 s；插件仅在回调中取一次 `node_->get_clock()->now()` 作为整帧时间戳，并在同一次 `rayShape->Update()` 中求所有射线。当前实现没有“本帧点分别采于前 100 ms”的物理时间窗口；0.1 s 只是消息周期和适配器的假设时间跨度。CSV 首列的单位仍未知，不能据此确定真实逐点时刻。
- 上述 bag 与原始覆盖数据均采集于修正前。修正后新 bag 才应包含 `offset_time` 字段，并将逐点采样时刻与点坐标关联；旧 bag 的时间值仍只是数组位置推算。

### 静止多帧覆盖检查

`bash sim/run_raw_lidar.sh --accumulate` 在静止 House 场景保存 100 个原始单帧，并生成 `raw_accum_valid_100.pcd` 与 `raw_coverage.json`。本次输出在 `sim/output/raw_EfB6917k/`。把有效回波按 10 cm 体素去重后，累积到第 1/10/30/100 帧时分别占 124/3518/5862/9660 个体素；第 100 帧仍比第 30 帧增加，说明非重复方向继续补充采样，但不能据此宣称房间表面已完整覆盖或地图精度达标。`bash sim/run_raw_lidar.sh --gui --accumulate` 使用独立 RViz 配置在固定的 `raw_lidar` 坐标系显示最近 10 秒原始有效回波；无 FAST-LIO2 或运动补偿。

下一步需用新的移动采集检查 `offset_time` 是否随帧内 Gazebo 仿真时刻递增，并与 IMU、真值时间对齐；还需评估 CSV 点序均匀映射对真实 MID-360 时序的近似误差。

来源：[MID-360 官方规格](https://www.livoxtech.com/cn/mid-360/specs)、[Livox 原始仿真插件](https://github.com/Livox-SDK/livox_laser_simulation)、[Livox ROS Driver 2 消息定义](https://github.com/Livox-SDK/livox_ros_driver2)、[FAST-LIO](https://github.com/hku-mars/FAST_LIO)。
- `mid360.xacro` 默认 `topic` 参数是 `mid360`，不是本项目契约 `/livox/lidar`；IMU 仍需由 Gazebo 独立插件发布到 `/livox/imu`。

## 备选核对

`vendor_audit/tfly6_ros2` 是 `Tfly6/Mid360_px4_sim_plugin` 的 `ros2` 分支，提交 `774212badac4d74b894dc06fa8e27043588482e0`。它确实有 `CustomMsg` 发布与 `offset_time` 写入，但 `package.xml` 声明依赖 `gz-sim8` 等 Gazebo Sim 8 库，当前机器的 Fortress 是 Gazebo Sim 6；不能作为 Classic 11 或 Fortress 6 的直接替换。该分支未见根目录 LICENSE 文件，仅包描述写 Apache-2.0，分发授权需进一步确认。

## 当前运行选择

`sim/run_plugin_demo.sh` 使用 Gazebo Classic 11、开源 TurtleBot3 Waffle Pi 车模、Livox 插件和 `plugin_mid360_adapter.py`。它已验收：插件与转换器每帧各 4000 点，同车 Gazebo IMU 接入 `/livox/imu`，FAST-LIO2 在 `sim/output/plugin_FkETkDrs/map_3d.pcd` 保存 101264 点，低速运动里程计校验通过。`run_demo.sh` 仍是独立的 Fortress 近似输入演示，两套脚本不同时运行。

该段中的转换器与运动验证结果对应修正前实现；之后录制的数据才会通过新的仿真时刻分批求交链路。

Classic 演示默认场景已换成开源 TurtleBot3 House，使用同一 Waffle Pi 和插件链路。`sim/output/plugin_TH3Ubz53/map_3d.pcd` 保存 100648 点，短程移动里程计 `(0.499, 0.102, 0.027)`，`/map_save` 成功。这个 PCD 只覆盖起点附近的局部区域。

另有完全隔离的 `sim/run_raw_lidar.sh`：车静止，只运行 Gazebo 插件与原始 PointCloud2 记录/显示节点；不启动 FAST-LIO2。它保存单帧 `raw_full_*.pcd` 和剔除无回波点后的 `raw_valid_*.pcd`，并可用 `--gui` 实时显示。`sim/output/raw_RAlIZAwD/` 的三帧原始点各 4000，分别有 1002、1645、2365 个有效回波。与 FAST-LIO2 输出的 `map_3d.pcd` 不应混称。
