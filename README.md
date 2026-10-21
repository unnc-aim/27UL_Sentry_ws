# sentry_ws

本工作空间保存哨兵机器人的底盘与云台控制、裁判通信、自瞄、Nav2 导航和比赛行为树。
当前实车流程使用 Odin1 的里程计与点云，沿用原比赛行为树完成前进、战斗、撤退、恢复及比赛结束处理。
快捷操作与底层服务入口见 [Makefile](Makefile)。

## 当前适配内容

- **双路感知**：`cloud_slam` 用于建图和定位，`cloud_raw` 经运动补偿、车体过滤和近点过滤后供实时避障使用。
- **地图与定位**：使用 `slam_toolbox` 建图；在已保存的二维地图上执行全图位置与朝向搜索，经新扫描复核后初始化 AMCL。
- **坐标与运动控制**：处理云台安装外参、测量时间对应的 TF 和速度方向转换；控制器增加目标前停车、障碍距离减速和弯道提前减速。
- **比赛流程**：正式入口为 `behavior` 包的 `competition_phase1`。等待定位时持续扫描云台并输出底盘零速；行进、撤退时云台朝运动方向观看。
- **地图选点与记录**：支持在独立 RViz 界面选点并自动保存 YAML，模拟裁判入口同时记录运行参数、终端输出和 rosbag。

当前比赛地图为 `odin_20261019_220142`，路线包含六个有序点，撤退按逆序返回。
控制器改动的构建、自动检查和已完成的现场测试见各包文档；新路线及速度调整仍需结合实际场地确认。

| 工作空间或模块 | 用途 |
|---|---|
| `~/sentry_ws` | 机器人控制、导航、比赛行为树与裁判通信 |
| `~/odin_ws` | Odin 驱动、双路点云适配、启动入口及选点工具 |
| `src/pb2025_sentry_nav` | 地图、AMCL 初始化、Nav2、速度转换与控制器子仓库 |
| `src/behavior` | 当前比赛行为树、六点路线和模拟裁判脚本 |
| `src/universal_controller` | 遥控与导航输入处理、底盘旋转和云台跟随 |

## 运行准备

以下命令按两个工作空间位于 `$HOME` 下编写，使用 ROS 2 Humble。
先完成工作空间编译及底层 EtherCAT、控制器等服务启动，再按实际遥控操作进入对应模式。
每个新终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/sentry_ws/install/setup.bash"
source "$HOME/odin_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 TZ=Asia/Shanghai
```

实车建图与导航使用同一个 Odin USB 设备，每次运行一套感知入口。
切换建图、导航时，先在原终端按 `Ctrl+C`，等进程退出后启动下一模式。
`mode:=mapping` 和 `mode:=navigation` 默认同时启动 Odin 驱动。
若出现 `LIBUSB_ERROR_BUSY`，检查另一套驱动或设备工具是否仍占用 USB。

## 1. 手动建图

终端 A，加载上述运行环境后执行：

```bash
ros2 launch odin_sentry_bringup odin_sentry.launch.py \
  mode:=mapping \
  use_robot_state_pub:=True \
  use_rviz:=True
```

先保持车辆静止，确认点云和地图持续更新，再使用 DT7 手动移动底盘、转动云台扫描场地。
当前建图设置按时间接收新扫描，底盘静止、仅云台转动时也会补充新方向。
建图时使用手动控制，并保持比赛行为树和模拟裁判程序处于退出状态。

### 保存地图

保持建图终端运行，另开终端 B 加载运行环境，再执行：

```bash
SENTRY_MAP_NAME="odin_$(date +%Y%m%d_%H%M%S)"
SENTRY_MAP_DIR="$HOME/sentry_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality"
mkdir -p "$SENTRY_MAP_DIR"

ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f "$SENTRY_MAP_DIR/$SENTRY_MAP_NAME" \
  --fmt pgm \
  --mode trinary \
  --free 0.196 \
  --occ 0.65
```

成功后生成同名 `.yaml` 和 `.pgm`，配套保留。`--free 0.196` 用于保留灰色未知区域。
本命令保存用于 AMCL 和导航的二维栅格地图；继续编辑建图轨迹所需的 `slam_toolbox` 位姿图需另行序列化保存。
关闭建图终端前确认文件已生成；同名保存会覆盖已有文件。

本文后续默认使用现有比赛地图。使用新图时，将各命令的 `map:=` 改为刚保存的 YAML 路径，并按新地图重新选点。
示例直接加载源码目录下的地图，保存后即可使用。

## 2. 地图选点模式

新终端加载运行环境，再切换到独立选点域：

```bash
export ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1

ros2 launch odin_sentry_bringup route_picker.launch.py \
  map:="$HOME/sentry_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/odin_20261019_220142.yaml" \
  output_file:="$HOME/odin_ws/routes/odin_$(date +%Y%m%d_%H%M%S).yaml"
```

该入口启动地图显示和点位记录，选点话题为 `/odin_route/goal_pose`。
每次使用新的输出文件名，已存在的文件会触发提示并结束启动。

1. 在 RViz 按 `G` 或选择 **2D Goal Pose**。
2. 在地图白色可通行区域按住左键拖动，设置位置与朝向。
3. 按路线顺序重复选点；每个有效点立即写入输出 YAML。
4. 完成后关闭 RViz，保存文件继续保留。

撤销最后一个点：在另一个已加载运行环境的终端执行：

```bash
export ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1
ros2 service call /odin_route/undo std_srvs/srv/Trigger '{}'
```

输出位置为 `~/odin_ws/routes/odin_<时间>.yaml`。
文件的 `waypoints` 按顺序保存 `x`、`y`、`yaw`，单位分别为米、米、弧度；地图路径相对路线文件保存。
当前比赛路线记录为 `~/odin_ws/routes/odin_competition_20261020_201546.yaml`。

### 将选点写入比赛行为树

选点工具保存路线文件，比赛行为树读取 XML 中的 `goals`。
更新前进和撤退点列的位置分别为：

- [competition_phase1.xml](src/behavior/behavior_trees/competition_phase1.xml)：前进路线。
- [competition_retreat.xml](src/behavior/behavior_trees/competition_retreat.xml)：撤退路线。

```xml
<SendNav2ThroughPoses
  goals="x1;y1;yaw1|x2;y2;yaw2|x3;y3;yaw3"
  action_name="navigate_through_poses"/>
```

当前流程前进依次经过六点，撤退按逆序返回。更换路线后核对两份 XML，并更新安装内容：

```bash
cd "$HOME/sentry_ws"
colcon build --symlink-install --packages-select behavior
```

随后在启动行为树的终端重新加载 `install/setup.bash`。
实车导航终端使用 `ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0`。

## 3. 启动导航与全局重定位

终端 1，加载运行环境后执行：

```bash
cd "$HOME/sentry_ws"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 TZ=Asia/Shanghai

ros2 launch odin_sentry_bringup odin_sentry.launch.py \
  mode:=navigation \
  use_robot_state_pub:=True \
  use_rviz:=True \
  map:="$HOME/sentry_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/odin_20261019_220142.yaml" \
  params_file:="$HOME/sentry_ws/log/behavior_speed_test/nav2_3p0_no_spin.yaml" \
  linear_speed_limit:=3.0 \
  angular_speed_limit:=3.0
```

`nav2_3p0_no_spin.yaml` 是本机保留的比赛速度配置，位于 `log/` 下。
部署到另一台主机时，一并迁移该文件，或将 `params_file` 指向确认过的同等配置。
控制器线速度上限为 `2.5 m/s`，速度平滑器 X/Y 上限为 `3.0 m/s`。
Odin 入口配置平移减速度为 `2.0 m/s²`；实际停车效果受底盘、地面和功率限制影响。

**导航开始输出日志后，及时启动下一节的比赛行为树。**
自动云台扫描由行为树负责，初始化时保持底盘静止。
输入等待、全图搜索和候选复核共用最多 `60 s`；满足条件后即可提供初始位姿并继续启动导航。
若显示初始化超时，重新启动导航与行为树这一组入口。

定位期间使用近期多方向扫描区分相似位置，连续三帧复核通过后向 AMCL 提交初始位姿。
随后 AMCL 持续定位。现场确认 RViz 中车辆位置、车头朝向和扫描与地图吻合后，再下发航点或模拟开赛。

## 4. 启动比赛行为树

终端 2，加载运行环境后执行：

```bash
cd "$HOME/sentry_ws"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 TZ=Asia/Shanghai

ros2 launch behavior pb2025_sentry_behavior_launch_new.py \
  params_file:="$HOME/sentry_ws/src/behavior/params/sentry_behavior_phase1.yaml"
```

启动成功会显示：

```text
Behavior Tree startup request accepted: competition_phase1
```

等待地图、定位和 Nav2 期间，行为树持续输出底盘零速、小陀螺零速和云台扫描指令。
初始化扫描角速度为 `0.5 rad/s`；进入前进或撤退阶段后，云台由导航速度方向控制。
正式执行的树为 `src/behavior/behavior_trees/competition_phase1.xml`。

比赛行为树与导航共同完成初始化，因此按“导航、行为树、核对定位、模拟裁判”的顺序启动。
真实裁判系统与模拟裁判测试分别使用各自的话题发布来源。

## 5. 启动模拟裁判完整流程

**此命令会发布开赛状态并触发实车执行路线。**
运行前完成地图定位核对，确认路线空间及遥控接管条件，并将真实裁判话题发布入口切换为退出状态。

终端 3 执行：

```bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 TZ=Asia/Shanghai
bash "$HOME/sentry_ws/src/behavior/scripts/run_competition_trial_recorded.sh"
```

脚本自动加载 ROS 和 `sentry_ws` 环境，记录运行参数、终端输出和 rosbag，随后调用 `run_competition_trial.sh`。
先发布准备阶段，等待 Nav2 和行为树待机输出，再自动发布 `game_progress=4` 开赛。
后续按反馈模拟低血量撤退、恢复、再次前进及比赛结束。
真实比赛时，由真实裁判通信节点提供对应状态。

模拟记录保存在 `~/sentry_ws/log/behavior_spin_detail/`。
结束测试时先遥控接管并停稳车辆，再依次结束模拟裁判、行为树和导航终端。

## 文件与记录位置

| 内容 | 路径 |
|---|---|
| 当前比赛地图 | `src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/odin_20261019_220142.{yaml,pgm}` |
| 新建地图 | `src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/odin_<时间>.{yaml,pgm}` |
| 早期 Odin 原始地图 | `~/odin_ws/maps/odin_20261019_220142/odin.{yaml,pgm}` |
| 选点文件 | `~/odin_ws/routes/odin_<时间>.yaml` |
| 当前比赛树配置 | `src/behavior/params/sentry_behavior_phase1.yaml` |
| 前进、撤退点列 | `src/behavior/behavior_trees/competition_phase1.xml`、`competition_retreat.xml` |
| 导航与初始化记录 | `log/nav_runtime/<启动时间>/`，其中 `localization.json` 记录搜索与复核结果 |
| 模拟裁判、参数及 rosbag | `log/behavior_spin_detail/` |

比赛详细流程见 [behavior README](src/behavior/README.md)。
Odin 安装、感知参数与测试记录见 [Odin 适配 README](../odin_ws/src/odin_sentry_bringup/README.md)。
控制器参数说明见 [全向路径控制器](src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/README.md)，
速度转换说明见 [fake_vel_transform](src/pb2025_sentry_nav/fake_vel_transform/README.md)。

## 安装 OpenVINO

<https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-apt.html>

```bash
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-get install gnupg
sudo apt update
sudo apt install openvino-2024.6.0
```

## MVS

<https://www.hikrobotics.com/cn2/source/support/software/MVS_Linux_STD_V4.6.3_260205.zip>
