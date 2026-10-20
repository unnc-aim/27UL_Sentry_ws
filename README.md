# sentry_ws

快捷操作请参考 [Makefile](./Makefile)


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

## 实车建图、重定位与点位保存

工作目录：`/home/soyo/sentry_ws`。以下命令使用默认空命名空间，运行前完成工作空间编译及真车底层服务启动。

以下以新地图 `field_20260925` 为例。复用已有地图时，将命令中的 `SENTRY_MAP_NAME` 改为对应名称，例如 `field_20260925`。

### 1. 启动建图

在终端 A 执行：

```bash
cd /home/soyo/sentry_ws
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True \
  use_rviz:=True
```

启动后先保持车辆静止，等待地图显示，再通过遥控移动完成场地扫描。

建图完成后，保持终端 A 运行，另开终端保存地图。

### 2. 保存地图

在终端 B 执行。每张新地图使用独立名称；同名保存会覆盖已有地图文件。

```bash
cd /home/soyo/sentry_ws
source install/setup.bash

SENTRY_MAP_NAME=field_20260925
SENTRY_MAP_DIR="$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality"

mkdir -p "$SENTRY_MAP_DIR"

ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f "$SENTRY_MAP_DIR/$SENTRY_MAP_NAME" \
  --fmt pgm \
  --mode trinary \
  --free 0.196 \
  --occ 0.65
```

生成文件：

```text
/home/soyo/sentry_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/field_20260925.yaml
/home/soyo/sentry_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/field_20260925.pgm
```

`.yaml` 保存地图分辨率、原点及图像路径，`.pgm` 保存栅格图像。两者配套保留。

`--free 0.196` 用于保留地图中的灰色未知区域。

确认保存成功后，在终端 A 按 `Ctrl+C` 结束建图，再启动导航定位。

### 3. 启动 AMCL 重定位与导航

在终端 A 执行：

```bash
cd /home/soyo/sentry_ws
source install/setup.bash

SENTRY_MAP_NAME=field_20260925

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:="$SENTRY_MAP_NAME" \
  map:="$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/${SENTRY_MAP_NAME}.yaml" \
  slam:=False \
  use_robot_state_pub:=True \
  use_rviz:=True \
  auto_global_localization:=True
```

该命令启动导航及 AMCL 自动全局初始化。当前启动入口为初始化脚本传入 `--seconds 15`。

这里使用地图 YAML 的完整路径，直接加载刚保存的地图文件。RViz 在下一节单独打开。

### 4. 打开地图界面并连续选点

保持第 3 节的导航进程运行，在终端 B 执行：

```bash
cd /home/soyo/sentry_ws
source install/setup.bash

ros2 launch pb2025_nav_bringup rviz_launch.py
```

图形化操作：

1. 确认显示的是目标地图，`Fixed Frame` 为 `map`。
2. 等待 Navigation 显示 `active`，点击 **Waypoint / Nav Through Poses Mode**。
3. 点击 **Nav2 Goal**，在目标位置按住左键并拖动，设置该点朝向。
4. 每次选点前重新点击 **Nav2 Goal** 或按 `G`，依次选出三个点。
5. 确认地图上出现 `wp_1`、`wp_2`、`wp_3`，保持 RViz 窗口和选点模式，执行下一节保存。

选点期间保持车辆急停。**Start Nav Through Poses** 和 **Start Waypoint Following** 用于开始实际导航。

实际导航前，确认 RViz 中机器人位置正确、激光与地图贴合。

### 5. 保存选点结果

选完三个点后，在终端 C 执行：

```bash
cd /home/soyo/sentry_ws
source install/setup.bash

SENTRY_MAP_NAME=field_20260925
mkdir -p waypoints

SENTRY_POINTS_FILE="$PWD/waypoints/${SENTRY_MAP_NAME}_points_$(date +%Y%m%d_%H%M%S).yaml"

ros2 topic echo /waypoints visualization_msgs/msg/MarkerArray \
  --qos-durability transient_local \
  --full-length \
  --once > "$SENTRY_POINTS_FILE"

echo "$SENTRY_POINTS_FILE"
```

命令收到当前点列后自动结束，并打印保存路径，例如：

```text
/home/soyo/sentry_ws/waypoints/field_20260925_points_20260925_153000.yaml
```

该文件是 `/waypoints` 的原始记录，保留位置、四元数朝向和点选顺序。

每个点包含箭头、圆点和文字标记。后续提取箭头标记，即可得到三个有序位姿，并转换为行为树使用的格式：

```xml
<SendNav2ThroughPoses
  goals="x1;y1;yaw1|x2;y2;yaw2|x3;y3;yaw3"
  action_name="navigate_through_poses"/>
```

`x、y` 单位为米，`yaw` 单位为弧度。现有行为树的录入位置：

```text
/home/soyo/sentry_ws/src/pb2025_sentry_behavior/behavior_trees/rmul_2025_reality.xml
```

点位保存与行为树录入是两个步骤。重新建图或调整地图原点后，按新地图重新选点。

### 文件位置

| 内容 | 保存位置 |
|---|---|
| 地图 YAML 和 PGM | `src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/` |
| 已选点位记录 | `waypoints/<地图名>_points_<时间>.yaml` |
| 导航运行日志 | `log/nav_runtime/<启动时间>/` |
| 自动初始化记录 | `log/nav_runtime/<启动时间>/localization.json` |
| 行为树巡航点 | `src/pb2025_sentry_behavior/behavior_trees/rmul_2025_reality.xml` |