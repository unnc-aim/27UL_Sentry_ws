# 2026-09-25 14:00 导航运动分析

数据：`/home/soyo/rosbag2_2026_09_25-14_00_13`，时长 216.996 秒。本文时间均从 bag 首条消息起算。

## 小陀螺

用户选择：导航行驶的小陀螺由行为树 `/cmd_spin` 单独控制。

本包收录的 `/cmd_vel_nav2_result` 和 `/cmd_vel` 各有 90 条，角速度均为 0；包中没有 `/cmd_spin` 记录。当前真实配置 `init_spin_speed=0`。RViz 导航目标本身只触发 Nav2 行驶，小陀螺需要行为树发布明确的旋转指令。

现有调用关系：行为树 `PublishSpinSpeed` 发布 `example_interfaces/msg/Float32`；`fake_vel_transform::cmdSpinCallback()` 保存旋转速度；`transformVelocity()` 将其加到导航角速度；Hub 的 NAVIGATION 分支使用最终 `TwistStamped.angular.z`。本次检查保留这些现有接口，测试覆盖平移中 `/cmd_spin=1.7` 和 `/cmd_spin=0`。

源码位置：

- `src/pb2025_sentry_behavior/behavior_trees/rmul_2025_reality.xml:21`：等待开始时发 0；`:201`：进入任务后发 7.0 rad/s。
- `src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp:72`：接收；`:134`：合成角速度。
- `src/universal_controller/src/hub/hub_arbitration.cpp:170`：导航输出。

遥控拨杆配置、解释器和 Hub 保留本次任务开始时的内容。上面的控制关系指 Hub 使用有效导航速度期间；导航速度超过 0.2 秒未更新时，现有 Hub 会转回 RC，RC 小陀螺按原有拨杆状态处理。

## 分段前进

对输入和输出按 bag 接收时间配对，允许间隔小于 0.02 秒，共得到 89 组互不重复的配对。其中 **35 组输入平移速度超过 0.01 m/s，输出三轴速度全部为 0**。

典型记录如下。年龄按当前记录时间减消息自身时间戳计算。

| bag 时间（秒） | `/odometry` 年龄（秒） | `/lidar_odometry` 年龄（秒） | 原始 LIO 年龄（秒） |
| --- | --- | --- | --- |
| 68.331 | 3.894 | 0.119 | 0.094 |
| 70.081 | 5.644 | 0.053 | 0.044 |
| 181.637 | 6.398 | 0.099 | 0.052 |
| 202.090 | 14.102 | 0.104 | 0.053 |

68.331 秒的输入速度模长为 0.833 m/s，输出为 0。`fake_vel_transform.cpp:100` 要求里程计年龄在 0.5 秒以内；79.835 秒还录到该节点等待新里程计的日志。0.5 秒检查继续保留。

点云记录也显示转换链更新落后：202.090 秒时，`/cloud_registered` 年龄为 0.052 秒，`/registered_scan` 为 0.952 秒，`/sensor_scan` 为 14.102 秒。

当前 Point-LIO 原配置逐点发布里程计；`loam_interface` 同时处理高频里程计和点云，`sensor_scan_generation` 再做配对。录到的 `/lidar_odometry` 为 80,281 条，`/cloud_registered` 为 818 条。原始里程计与点云使用不同的时间戳取值，增加了中间处理和配对的负担。

已采用 Point-LIO 自带的 `odometry.publish_odometry_without_downsample=False`：每帧点云配一帧里程计，并统一使用 `lidar_end_time`。该开关只改变发布方式，Point-LIO 内部估计继续使用原算法。

对应源码：`point_lio/src/laserMapping.cpp:265` 选择时间戳；`:965` 每帧发布里程计；`:984` 发布点云。修改文件为 `pb2025_nav_bringup/config/reality/nav2_params.yaml:78`。

35 组零输出及转换链旧数据由 bag 支持；同步和处理负担是结合源码得到的定位。包记录也可能缺帧，消息总数只用来描述录到的数据。整车运行下的更新连续性留待新版本实车测试。

## 移动过量

首条 `/odometry` 的平面速度模长达到 **3,653,285.23 m/s**，属于计算异常。旧 `publishOdometry()` 从首次回调的极小处理时间求差分，后续也使用回调间隔；成批处理消息时，速度会失真。旧代码还直接把父坐标系速度填入 `child_frame_id` 对应的速度字段。控制器启用了按速度调整前视距离，因此异常速度会影响路径跟踪。

已修正 `sensor_scan_generation/src/sensor_scan_generation.cpp:132`：首帧速度为 0；按消息测量时间求差分；速度旋转到子坐标系；角速度使用最短四元数旋转；历史状态放入节点实例。对应头文件 `sensor_scan_generation/include/sensor_scan_generation/sensor_scan_generation.hpp:74` 增加时间、位姿和首帧状态。

旧真实配置的接近目标速度下限为 0.5 m/s，减速距离为 0.5 m，最高平移速度为 3 m/s；bag 参数事件确认了最高速度 3 m/s，录到的速度指令模长最高为 2.461 m/s。这些配置与更新间断共同增加停车误差。当前调整为：

- 平移速度上限 1.0 m/s；速度平滑器 x/y 上限同步调整。
- 距目标 1.0 m 开始按距离减速，接近速度下限 0.05 m/s。
- PID 平移速度标量下限 0，平移方向继续由目标相对位置决定。

位置到达容差继续为 0.25 m。上述修改解决已发现的速度计算问题，并提供较缓的接近目标参数；实际停车误差和底盘制动效果通过实车复测量化。

## 修改和验证

生产文件共 3 个：

1. `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`：第 78、410、603 行附近。
2. `src/pb2025_sentry_nav/sensor_scan_generation/src/sensor_scan_generation.cpp`：第 132 行附近。
3. `src/pb2025_sentry_nav/sensor_scan_generation/include/sensor_scan_generation/sensor_scan_generation.hpp`：第 74 行附近。

扩展已有 `scripts/real_acceptance/test_velocity_adapter.py:136`，新增生产 cpp/py 文件数为 0。

`sensor_scan_generation`、`pb2025_nav_bringup` 编译成功，详见 `build.log`。隔离 ROS 域 179 的测试通过 7 组检查，详见 `test.log`，覆盖速度坐标转换、平移时旋转指令、旧数据停车、里程计时间和速度、原生后退方向、后退遇障停车、速度消息类型一致性。

复现测试：

```bash
cd /home/soyo/sentry_ws
source install/setup.bash
SENTRY_TEST_OUTPUT=scripts/real_acceptance/results/motion_review_20260925_140013 python3 scripts/real_acceptance/test_velocity_adapter.py
```

加载修改需要重新启动真实导航。下一次实车复测记录连续直线行驶和目标附近停车，并检查 `/odometry`、`/sensor_scan`、`/cmd_vel_nav2_result`、`/cmd_vel`、`/cmd_spin` 与 `/chassis_command`。
