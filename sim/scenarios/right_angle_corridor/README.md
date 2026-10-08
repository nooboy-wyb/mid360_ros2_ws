# MID360 单程直角走廊场景

这是独立场景分类 `right_angle_corridor`，用于测 FAST-LIO2 单程前端定位。走廊两臂均约 4 m 宽，路线为：从水平臂直行约 4 m → 原地左转 90° → 竖直臂再直行约 4 m → 停车。两臂和拐角布置 5 个半径/高度不同的静态圆柱体，避免场景只有无限平行墙面。

Gazebo 真值仅用于路线控制和离线评估；FAST-LIO2 仍只接收 MID360 点云与 IMU。路线控制器用真值让小车到达指定位置和角度，不把真值喂给 FAST-LIO2。`route_trace.csv`、`route_events.csv` 会记录实际运动，不假设机器人准确走过命令轨迹。

## 运行

```bash
bash sim/scenarios/right_angle_corridor/run_test.sh --gui
```

每次运行默认选择独立 `ROS_DOMAIN_ID`（即使当前终端已导出固定的 `ROS_DOMAIN_ID` 也会隔离），启动适配器后确认 `/livox/lidar` 和 `/livox/imu` 各只有一个发布者，避免旧适配器残留导致同一帧被重复送入 FAST-LIO2。需要固定 domain 时可预先设置 `RIGHT_ANGLE_ROS_DOMAIN_ID`。

手动录制完成后，可以对同一份传感器输入重新运行 FAST-LIO2 并重新计算误差：

```bash
bash sim/scenarios/right_angle_corridor/replay_fastlio.sh sim/output/right_angle_corridor/manual_XXXXXXXX
```

重放会跳过原 bag 中已有的 `/Odometry`，并在 `manual_XXXXXXXX/replay_时间戳/` 保存新 FAST-LIO2 里程计和评估结果。

省略 `--gui` 时无界面运行。默认 LiDAR/IMU 帧头偏移为 0；可选 `--lidar-shift-ms 10` 复测当前候选 +10 ms。每次运行写入独立的 `sim/output/right_angle_corridor/run_时间戳/`，包含 world SDF、同步 rosbag、FAST-LIO2 地图、真值与估计轨迹、指标和对比图。bag 已录 `/clock`，重放时不要再加 `--clock`。

主要文件：

- `trajectory_bag/`：点云、IMU、FAST-LIO2 `/Odometry`、Gazebo 真值、控制与 `/clock`。
- `metrics.json`、`evaluation.md`：定位误差、路程比、转角和 RPE。
- `localization_plot.png`：FAST-LIO2 轨迹与 Gazebo 真值对比及误差曲线。
- `fastlio_map.pcd`：FAST-LIO2 保存的地图，可用 `bash sim/view_fastlio_map.sh <PCD路径>` 查看。`view_raw_map.sh` 只用于 `raw_accum_valid_odom.pcd`，两种 PCD 字段格式不同。

单次路线不回到起点，所以适合看没有闭环约束时的局部里程计和转角误差，不能用来评估回环效果。

## 评估方式

先区分**绝对精度**和**相对稳定性/漂移**。FAST-LIO2 的里程计坐标原点通常不等于 Gazebo 世界原点；比较两者时只能使用一次固定坐标变换，不能沿轨迹反复重对齐。

### 静止测试

- 记录 Gazebo 真值位置，确认传感器在测试期间确实静止。
- 单独检查启动和初始化阶段。只有在测试开始前约定好预热时长或收敛判据后，才从稳定时刻开始统计；同时保留并报告启动瞬态，不能为改善结果而事后挑选时间段。
- 在稳定时刻把 FAST-LIO2 位姿固定为参考，计算后续相对位移的 RMSE、中位数、p95、最大值、末端位移，以及 x/y/z 各轴标准差、范围和随时间的趋势。这些指标反映静止时的位置抖动和漂移。
- 静止场景不以逐帧累加的轨迹长度作为定位精度。`fastlio_path_length_m` 是相邻 `/Odometry` 位置差的三维范数之和；它会把每帧抖动累加，受输出频率影响，不能解释为静止传感器实际走过的路程。

### 移动测试

- 若机器人先静止初始化再出发，将预热段单独报告，并在固定起始位姿处开始路线评分；若启动后立即移动，则启动阶段也属于任务，应纳入总误差，并另外报告收敛时间。移动路线中途和终点不随意裁掉。
- 有已知初始位姿时，只用起点确定一次估计坐标系到真值坐标系的变换，之后保持不变。报告平移 APE 的 RMSE、中位数、p95和最大值；可同时报告旋转误差。
- 报告 1/2/5 秒或固定距离间隔的 RPE，观察短期里程计漂移。路线长度可与 Gazebo 真值路线长度比较，但作为辅助指标，不替代 APE/RPE。
- 全轨迹 SE(3) 对齐 APE 只说明轨迹形状对齐后的误差，会消除整体平移和旋转偏差；应明确标注，不能单独当作绝对定位精度。也不要对每一段或每个时刻重新对齐。

`evaluate_run.py` 当前生成的全局 SE(3) 对齐 APE、首位姿锚定误差和累计路径长度是原始诊断项。解读静止 bag 时要先排除已确认的初始化瞬态，并按上面的固定稳定参考方式计算；不要把累计路径长度当作静止漂移或绝对误差。

### 地图精度

- RViz 叠加检查用于发现墙面重影、弯曲、厚度异常、地面倾斜和缺失区域，属于定性检查。
- 定量评估需要参考地图。该场景可由 SDF 中已知尺寸和位置的墙、地面、圆柱地标生成参考点云，并在已知 Gazebo 世界坐标中比较；报告地图点到参考平面的 RMSE、p95，以及参考表面的覆盖率/缺失率。不要对每个墙面单独配准，否则会掩盖整图偏移和形变。
- 静止雷达 bag 主要验证静态环境下的点云一致性和建图稳定性；要评价移动建图，还需让雷达沿路线运动，并与同一场景的参考几何比较。地图看起来重合不能单独证明厘米级精度。

### 跨 run 横向比较

把同一类测试的 run 目录一起交给汇总脚本；静止测试统一用 10 秒预热（需要改时必须对所有待比较 run 使用同一个值），移动测试只评分 `route_events.csv` 标注的路线阶段：

```bash
python3 sim/scenarios/right_angle_corridor/compare_runs.py \
  sim/output/right_angle_corridor/run_A \
  sim/output/right_angle_corridor/run_B \
  --warmup-seconds 10 \
  --output sim/output/right_angle_corridor/comparison.csv
```

脚本按真值路程把静止和移动 run 分开标注。静止行比较预热后的固定参考位移 RMSE、p95、最大值、末端变化和漂移率；移动行比较路线阶段的全局 SE(3) 对齐 APE、1 秒/5 秒 RPE 和路线长度误差。比较 FAST-LIO2 参数时，应让各版本重放同一份传感器 bag；比较传感器仿真版本时，应使用相同路线和场景。只在同类测试、相同路线和相同配置基线之间横向比较；全局对齐 APE 仍只表示轨迹形状误差，不表示绝对坐标精度。生成的 CSV 保存每个 run 的指标，累计路径长度不列作静止精度指标。
