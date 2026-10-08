# 小场景 MID360 风格点云 + FAST-LIO 建图验证

本阶段使用本机 `ros-humble-turtlebot3-gazebo` 中的 TurtleBot3 World 静态模型作为轻量场景。该模型包含 `wall.dae`、`hexagon.dae` 两个 Collada mesh，模型目录约 56 KB；墙体 mesh 缩放后约 5.7 × 6.6 m，约 38 m²。ROS 包声明 Apache 2.0。Gazebo Fortress 的 world、简化差速底盘、3D LiDAR 和 IMU 配置在 `worlds/mesh_lab.sdf`。

**边界：** Gazebo 3D LiDAR 是 MID360 的近似输入。`approx_mid360_adapter.py` 将一次扫描转换为 Livox `CustomMsg`，保留消息类型、topic、线号和时间基。源点云只有帧时间；逐点 `offset_time` 按列号在 100 ms 内合成，不能证明真实 MID360 非重复 pattern 或运动去畸变性能。

## 运行

先确保已在此工作区执行过 `colcon build --symlink-install --packages-up-to fast_lio`，然后：

```bash
cd /home/wyb/workspace/mid360_ros2_ws
bash sim/run_demo.sh
```

脚本无 GUI 运行约 25 秒，低速移动底盘，调用 `/map_save`。每次输出保存在独立的 `sim/output/run_*/`，包括日志、里程计文本和 `map_3d.pcd`。脚本校验地图非空和短程里程计无明显发散；失败记录仍保留。一次通过的运行生成了 168350 个 PCD 点。

启动文件只引用系统安装的 TurtleBot3 World 模型，没有复制其 mesh。FAST-LIO 输入配置在 `fastlio_sim.yaml`；底盘尺寸和雷达近似扫描参数可在 world SDF 中修改。

## Livox 插件 + 开源 Waffle Pi 车模

`run_plugin_demo.sh` 使用 Gazebo Classic 11 的 Livox 插件和本机安装的 TurtleBot3 Waffle Pi 完整底盘模型。`create_classic_world.py` 从系统安装的 Waffle Pi SDF 生成场景，保留原车身 mesh、轮组和差速驱动，移除原厂 2D 雷达，把 MID360 插件与 Gazebo IMU 装在同一安装位。车模来源为 [ROBOTIS turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)，Apache 2.0。

```bash
cd /home/wyb/workspace/mid360_ros2_ws
bash sim/run_plugin_demo.sh
bash sim/view_plugin_map.sh
bash sim/view_gazebo.sh
```

`view_plugin_map.sh` 只在 RViz 中显示最新一次插件演示保存的 PCD；`view_gazebo.sh` 显示 Waffle Pi 车模和当前 Gazebo 场景。两种查看入口互相独立。此前小场景运行保存了 101264 点的 PCD，位置为 `sim/output/plugin_FkETkDrs/map_3d.pcd`。插件只发布 `PointCloud2`，`plugin_mid360_adapter.py` 将其转为 `CustomMsg`；逐点时间是推算值，不能验证真实 MID360 运动畸变。

### 当前默认场景：TurtleBot3 House

Classic 插件演示现在默认加载系统安装的 [TurtleBot3 House](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) 室内场景，约 15 × 10 m，包含多房间墙体及家具。Waffle Pi 使用官方 House 启动位置 `(-2, -0.5)`。场景引用的四种 Gazebo 家具模型从 [osrf/gazebo_models](https://github.com/osrf/gazebo_models) 稀疏下载到 `vendor_gazebo_models/`，保留其 CC BY 3.0 `LICENSE`；`TurtleBot3 House` 本体遵循 ROBOTIS 仓库的 Apache 2.0 许可。生成场景时将默认地面和太阳替换为本地 SDF 对象，避免启动时访问在线模型库。

```bash
cd /home/wyb/workspace/mid360_ros2_ws
bash sim/run_plugin_demo.sh
bash sim/view_plugin_map.sh
bash sim/view_gazebo.sh
```

最新 House 验收结果是 `sim/output/plugin_TH3Ubz53/map_3d.pcd`，100648 点，约 3.1 MB。当前脚本只让车低速移动约 10 秒，地图覆盖起点附近，不代表整栋 House 已被完整扫描。若要回到此前小场景，可设置 `MID360_SCENE=lab` 再运行演示或 Gazebo 查看脚本。

## 隔离路线：只看 Livox 插件原始点云

先用这条路线核对单帧点云。它只启动 Gazebo Classic、TurtleBot3 House、静止的 Waffle Pi、Livox 插件和原始点云记录节点；**不启动** `plugin_mid360_adapter.py`、FAST-LIO2 或地图融合。插件原始话题仍是 `/mid360_plugin/points`，记录节点把其中无回波/无效点剔除后发布 `/raw_livox/valid_points` 供 RViz 实时观察。点的 XYZ 和 intensity 保持插件原值，显示时不做坐标配准或去畸变。

```bash
cd /home/wyb/workspace/mid360_ros2_ws
bash sim/run_raw_lidar.sh         # 无 GUI，采集并校验 3 帧
bash sim/run_raw_lidar.sh --gui   # Gazebo + RViz 实时看单帧；关闭 RViz 退出
bash sim/run_raw_lidar.sh --accumulate         # 静止采集 100 帧，输出累积 PCD 和覆盖统计
bash sim/run_raw_lidar.sh --gui --accumulate   # RViz 显示最近 10 秒的原始帧累积
```

每次输出在独立的 `sim/output/raw_*/` 目录。`raw_full_001.pcd` 保留插件原始 4000 点（包括零距离/无回波点）；`raw_valid_001.pcd` 只保留有效回波，适合直接打开查看。`raw_001.json` 记录时间戳、源帧号、插件帧序号、CSV 首列的原始范围、原始点数和有效点数。CSV 首列单位尚未核实，不能当成逐点纳秒时间。默认跳过前 20 帧，之后保存连续 3 个**独立单帧**；它们不叠加。一次静止 House 采集在连续三帧得到 4000/4000/4000 个原始点，其中有效回波分别为 1002/1645/2365 个。场景静止但 MID360 CSV 扫描方向按帧变化，单帧可见表面会变化。时间链路的实测和限制见 [插件审计](plugin_audit.md#csv-与仿真时间实测)。

与上面的原始路线隔离的是 `run_plugin_demo.sh`：它才会启动车辆运动、`CustomMsg` 转换、IMU 输入、FAST-LIO2，并输出融合地图 `map_3d.pcd`。要评估算法前，先看原始帧与实时话题是否符合预期。

累积模式的 `raw_accum_valid_100.pcd` 只把静止车体坐标系中的有效回波拼接起来，不是 FAST-LIO2 地图；`raw_coverage.json` 给出第 1、10、30、100 帧的 10 cm 占据体素数。一次 House 采集在这些检查点分别为 124、3518、5862、9660，说明覆盖持续增加，但尚不能断言完整。CSV 首列单位未确认，插件全部射线在每次 10 Hz 更新中一次求交；消息间隔 0.1 s 不等于逐点采样跨度 0.1 s，详见[插件审计](plugin_audit.md#csv-与仿真时间实测)。
