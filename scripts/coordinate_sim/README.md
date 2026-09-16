# 地图坐标一致性仿真

本目录独立运行 Gazebo Fortress、全局 AMCL 定位和现有
`pb_omni_pid_pursuit_controller`。不启动 EtherCAT、实车 Hub、发射器或裁判节点，
不修改实车自启配置。

## 本机验证结果（2026-09-16）

| 初始朝向 | 导航附加旋转 | east 误差 | north 误差 | home 误差 | 结果 |
| --- | --- | --- | --- | --- | --- |
| 1.2 rad | 0 rad/s | 0.126 m | 0.155 m | 0.181 m | 通过 |
| -2.0 rad | 0.3 rad/s | 0.153 m | 0.103 m | 0.186 m | 通过 |

两组均从全局粒子初始化定位，没有传入初始地图位姿；底盘启动旋转累计均超过
`4π`，云台保持扫描。第二组导航期间底盘真值累计旋转约 10.785 rad。
最小车体中心到障碍几何的距离分别为 0.701 m、0.541 m。

原始证据见 `results/*/result.json`、`result.csv`、`run_config.json`，轨迹图见
`results/trajectories.png`。3 项几何测试以及 ROS 接口测试通过；接口测试覆盖
初始无 TF、TF 过期、指令超时、任务心跳超时、里程计中断时的零速度输出。

## 运行

在本机桌面的终端、工作区根目录执行（通常约 2–4 分钟，取决于仿真实时率）。
同一个命令会启动 Gazebo 图形界面、仿真服务、定位和导航任务，无须再开终端连接界面：

```bash
source install/setup.bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 \
ROS_LOG_DIR=/tmp/sentry-frame-sim/log IGN_PARTITION=sentry-frame-sim \
ros2 launch ./scripts/coordinate_sim/launch.py \
  spawn_yaw:=1.2 output_dir:=/tmp/sentry-frame-sim/yaw_1_2
```

另一启动朝向，导航途中持续旋转底盘：

```bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 \
ROS_LOG_DIR=/tmp/sentry-frame-sim/log IGN_PARTITION=sentry-frame-sim \
ros2 launch ./scripts/coordinate_sim/launch.py \
  spawn_yaw:=-2.0 nav_spin_speed:=0.3 output_dir:=/tmp/sentry-frame-sim/yaw_minus_2
```

一次只运行一个实例。任务结束生成 `output_dir/result.json`，检查 `passed`，
然后 Ctrl-C 关闭仿真。中途关闭/崩溃不算通过。启动参数 `spawn_yaw` 只传给
Gazebo 的机器人生成服务，不传给定位器。

界面打开后，车辆自动执行启动旋转、全局定位和三个地图目标的导航。
关闭图形窗口只关闭显示；在启动终端按 Ctrl-C 才会关闭整套仿真。
无桌面或批量测试时，在上述启动命令末尾添加 `gui:=false`，仅运行仿真服务。
默认 `gui:=true` 需要有效的桌面显示环境；普通无图形转发的 SSH 终端不能显示窗口。

## 实际验证范围

1. `scenario.py` 由同一组非对称障碍几何生成场景、预建二维地图及命名目标。
   地图在机器人启动前生成；运行时不重新建图、不发送 `/initialpose`。
2. 原工作区 `simulation_robot` 模型及仿真底盘控制器被复用。删除灯条装饰插件
   （其在本机无界面渲染中崩溃）、发射插件和本测试不用的传感器。
3. AMCL 在整张地图的空闲区域初始化粒子，使用实际 Gazebo 激光扫描匹配。
   Gazebo 真值转换为以第一次观测为原点的**相对理想里程计**。
   初始地图位置/朝向不注入定位器。
4. 云台执行两段 13 秒的旋转目标，中间停 2 秒；另增加底盘按里程计累计转过
   `4π` 的压力测试。该云台目标使用仿真位置接口，不等同于实车 Hub 的扫描 PID。
5. 协方差、扫描端点与地图的一致性及时间戳连续满足条件后，才放行 Nav2。
   依次发送地图内 `east`、`north`、`home` 目标。整个导航期间云台继续旋转。

定位可信度阈值是本测试场地的工程门控，并非“任意地图全局定位必然正确”的保证。
对称、重复或缺少特征的场地仍可能无法唯一定位；失败时停止，不能用小协方差
代替实地匹配验证。本例不会用真值帮助定位收敛。

## 坐标约定

- 地图目标属于 `map`。转动不会重设地图原点或方向。
- AMCL 是唯一 `map -> odom` 发布者。
- 仿真桥是唯一 `odom -> base_footprint` 发布者。
- 机器人状态发布器从真实仿真关节生成云台 TF。
- Nav2 所有基座配置均使用 `base_footprint`；不再经过 `gimbal_yaw_fake`。
- 导航输出是底盘系速度。现有 Gazebo 接口接收云台系速度，因此桥只执行一次
  `base_footprint` 到 `gimbal_yaw` 的旋转；现有 Gazebo 控制器再转回底盘系。
- 缺失/过期的云台 TF 不允许使用单位变换替代。命令 0.3 秒超时或任务心跳
  0.5 秒超时，桥输出零速度。任务失去定位可信度，取消目标并关闭运动门控。

`result.json` 的到达误差使用独立 Gazebo 世界真值评估，不使用 AMCL 自己的
“到达”判断代替实测位置。目标误差阈值为 0.35 米；障碍物中心距离阈值为
0.32 米。后者是几何间距回归，不是 Gazebo 接触传感器的无碰撞证明。

## 地图选点与实车迁移

本回归的三个目标由地图场景提供，不需要每次手输坐标。实际比赛应将预建地图
与命名目标文件一起管理，在该地图上选点保存；运行时只选择目标名称。
初始位姿由定位器求解，目标点由任务决定，两者不能混为一谈。

当前测试**没有验证** Point-LIO 漂移、实车雷达安装外参、云台编码器零位、轮向
零位、实车 Hub 的速度取反约定，不能据此直接认定比赛自启已修好。
将该约定接入实车之前，需要：

1. 补齐真实关节 TF 或让导航输出直接使用底盘系，避免两套云台角度参与补偿。
2. 将底盘系速度与 LK 电机实际轴向校准，实测正 X、正 Y、正 yaw。
3. 在实车地图上选择可靠的定位器及可信度门控；现有 small_gicp 是局部配准，
   不能仅把初始猜测设成零就宣称支持任意起始姿态的全局定位。
4. 按实际比赛入口接入启动动作完成信号和目标命名表，并验证定位丢失停车。

无场地时，可先在当前房间建立自己的实车地图测试上述内容；地图坐标与地理方向
无须一致，但必须与激光匹配和目标点来自同一张地图。

几何回归：

```bash
python3 scripts/coordinate_sim/test_geometry.py
```

ROS 故障回归（独立 `/coordinate_bridge_test` 命名空间，不接执行器）：

```bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/sentry-frame-sim/log \
IGN_PARTITION=sentry-frame-sim python3 scripts/coordinate_sim/test_bridge.py
```

轨迹保存在 `result.csv`；绘图：

```bash
MPLCONFIGDIR=/tmp/sentry-frame-sim/matplotlib python3 -s \
  scripts/coordinate_sim/plot_results.py \
  /tmp/sentry-frame-sim/yaw_1_2 /tmp/sentry-frame-sim/yaw_minus_2 \
  --output /tmp/sentry-frame-sim/trajectories.svg
```

`python3 -s` 使用本机系统 Matplotlib/NumPy，避免用户目录 NumPy 2 与系统
Matplotlib 的二进制版本冲突。仿真本身不依赖 Matplotlib 或 SciPy。
