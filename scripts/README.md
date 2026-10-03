# 临时测试脚本与联调工具

本目录保存本地联调、实车测试、仿真验证和诊断脚本，以及备用启动工具、测试地图和历史结果。
这些脚本通过各自入口按需执行。当前常规车辆运行使用 `src/` 中对应 ROS2 包提供的程序。

## 目录内容

| 类别 | 文件或目录 | 用途 |
| --- | --- | --- |
| 实车测试 | `test_simple_nav*`、`run_simple_nav_test*`、`test_competition*`、`run_test_competition.sh`、`test_stationary*`、`test_mobility.sh` | 往返导航、云台扫描、自瞄开关、底盘自旋和整车功能测试 |
| 备用 Python 比赛入口 | `run_competition.sh`、`competition_launch.py`、`competition_controller.py` | 使用在线建图与 Python 任务流程组织导航、战斗和撤退 |
| 建图保存工具 | `run_reality_slam_with_autosave.sh` | 启动建图，在正常退出流程中保存栅格地图、点云和日志 |
| 坐标与导航仿真 | `coordinate_sim/` | Gazebo 场景、接口转换、定位导航任务、自动检查和结果绘图 |
| 实车诊断与试验 | `real_acceptance/` | 独立测试环境、数据采集、运动试验、标定、选点、路径检查与历史记录 |

`coordinate_sim/` 的 Python 脚本包括 `launch.py`、`scenario.py`、`bridge.py`、`mission.py`、
`test_bridge.py`、`test_geometry.py` 和 `plot_results.py`。

`real_acceptance/` 中的 `motion_probe.py`、`navigation_trial.py`、`map_test.py` 等文件用于独立试验。
`gimbal_follow.py`、`speed_profile.py`、`obstacle_hold.py` 等文件服务于这些试验程序。
`test_*.py` 用于开发验证，`maps/` 保存测试地图和目标点，`results/` 保存历史报告、日志与图像。
更详细的使用背景见 [real_acceptance/README.md](real_acceptance/README.md) 和
[coordinate_sim/README.md](coordinate_sim/README.md)。

## 当前车辆运行入口

| 功能 | 当前实现位置 |
| --- | --- |
| 实车导航启动 | [rm_navigation_reality_launch.py](../src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py) |
| AMCL 初始化 | [amcl_initializer.py](../src/pb2025_sentry_nav/pb2025_nav_bringup/scripts/amcl_initializer.py) |
| 导航云台朝向跟随 | [gimbal_controller.cpp](../src/universal_controller/src/controllers/gimbal_controller.cpp) 和 [hub_arbitration.cpp](../src/universal_controller/src/hub/hub_arbitration.cpp) |
| 比赛行为树 | [rmul_2025_reality.xml](../src/pb2025_sentry_behavior/behavior_trees/rmul_2025_reality.xml) |
| 基础服务配置 | [ros2_services.yaml](../ros2_services.yaml) |

AMCL 初始化程序原名为 `localization_probe.py`，现已迁入 `pb2025_nav_bringup`，随导航包安装。
导航 launch 通过包名和可执行文件名调用它。当前场地地图 `field_20260925.yaml` 位于导航包的
`map/reality/` 目录，常规导航运行日志写入工作区的 `log/nav_runtime/`。

备用 Python 比赛入口采用 `slam=True` 在线建图；当前 XML 比赛行为树由
`pb2025_sentry_behavior` 包负责启动。两套流程通过各自入口使用。

## Git 管理方式

根目录 [.gitignore](../.gitignore) 对以下脚本设置了忽略规则：

```gitignore
/scripts/*.py
/scripts/*.sh
/scripts/coordinate_sim/*.py
/scripts/real_acceptance/*.py
/scripts/real_acceptance/*.sh
```

这些测试、诊断和备用启动脚本保留在本地，Git 跟踪已移除。
说明文档、测试配置、地图和历史结果按照各自现有的跟踪状态保留。
后续用于常规车辆启动的程序应放入 `src/` 下对应的 ROS2 包，并通过包的安装配置管理。

## 执行前核对

部分脚本会发布底盘、云台或自瞄指令，复制测试配置，或启停 ROS 服务。
部分 `run_*` 测试入口结束时还会执行 `make restart`。
执行前阅读脚本动作和参数，确认现场遥控、急停状态及当前启动的程序。
