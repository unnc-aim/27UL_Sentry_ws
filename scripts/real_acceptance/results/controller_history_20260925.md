# 遥控器组合与控制器修改历史

查询日期：2026-09-25。范围：本地 `universal_controller` 当前哨兵分支的完整主线、相关合入改动、保存的旧状态，以及前身 `sentry_controller` Python 仓库。以下描述各版源码；实车采用版本以当时安装与启动记录为准。

本次只新增这份说明。控制器、导航配置和机器人进程保持本次查询开始时的内容。

## 阅读约定

- 左、右均指遥控器三档拨杆。原始数值：上 1，中 3，下 2。
- 按 NDJ 为主输入、连接正常、键鼠释放说明。另一图传遥控器作为主输入时，Hub 使用其开关状态。
- “自瞄开”表示左拨杆的自瞄许可；实际接管还需要有效视觉数据。导航期间的特殊处理在各版下方单列。
- “拨弹开”表示拨弹请求；电机执行还受本版火控检查影响。
- 表内小陀螺描述 RC 产生的旋转。导航旋转和行为树 `/cmd_spin` 在各版下方单列。

## 一、当前 C++ 控制器的六套组合定义

### 1. 2026-03-25 03:04 至当日 23:22 前：直接读取拨杆

对应提交：`fix: interpreter entry linking and msg spec` 起，配置文件引入之前。

| 左拨杆 | 右拨杆 | 源码选择的模式 |
| --- | --- | --- |
| 上 | 上 | 导航选择；RC 三轴速度清零 |
| 上 | 中 | 导航选择；RC 三轴速度清零 |
| 上 | 下 | 手动平移；进入左上时切换小陀螺 |
| 中 | 上 | 导航选择；RC 三轴速度清零 |
| 中 | 中 | 导航选择；RC 三轴速度清零 |
| 中 | 下 | 手动平移；保留小陀螺切换状态 |
| 下 | 上 | 急停 |
| 下 | 中 | 急停 |
| 下 | 下 | 急停 |

这个阶段的自瞄、发射由鼠标按钮产生。手动分支中摩擦轮默认开启。导航分支提前返回，其余字段沿用前次值。

这里“导航选择”指解释器置位 `navigation_enabled`；当时 Hub 的底盘分支仍以 Action/RC 为主。2026-03-27 22:38 的提交才接上导航速度订阅和 NAVIGATION 分支。

小陀螺采用切换记忆：进入左上切换一次，回到左中保留该状态；键盘 V 同样切换。左下输出停止，内部切换状态留存。

更早的 03-25 02:31 至 02:43 草稿使用 `gear_switching`、`pause_button`、`right_custom_button`、`left_custom_button` 等旧字段；03:04 才改为本机 NDJ 的 `left_switch/right_switch`。旧字段草稿的物理拨杆对应资料待补。

### 2. 2026-03-25 23:22：首版 YAML 定义

提交：`feat: add configurable ndj_definition`。文件当时使用 `left_trigger/right_trigger`。

| 左拨杆 | 右拨杆 | 模式 | 摩擦轮 / 拨弹请求 |
| --- | --- | --- | --- |
| 上 | 上 | 导航选择，自瞄开 | 开 / 连发请求开 |
| 上 | 中 | 导航选择，自瞄开 | 开 / 连发请求开 |
| 上 | 下 | 手动，自瞄开；小陀螺按切换记忆 | 关 / 关 |
| 中 | 上 | 导航选择，自瞄关 | 开 / 连发请求开 |
| 中 | 中 | 导航选择，自瞄关 | 开 / 连发请求开 |
| 中 | 下 | 手动，自瞄关；小陀螺按切换记忆 | 关 / 关 |
| 下 | 上 | 急停 | 关 / 关 |
| 下 | 中 | 急停 | 关 / 关 |
| 下 | 下 | 急停 | 关 / 关 |

左中拨上执行 `spin_mode.toggle`。开机直接处于左上时，首次消息只执行档位动作，切换动作从后续有效相邻拨动开始。

这个版本的右上拨中有一次额外动作：该帧把导航、摩擦轮和连发请求都清零；下一帧右中档位动作又将它们开启。因此该组合存在一帧的切换差异。此时 Hub 的导航输入还处于前述预留阶段。

### 3. 2026-03-26 16:06 至 2026-04-04 02:42 前：右中导航并开摩擦轮

起点提交：`feat: add vtm ctrl, update schema`。03-26 22:37 将配置键改名为 `left_switch/right_switch`，九种组合沿用。

| 左拨杆 | 右拨杆 | 模式 | 摩擦轮 / 拨弹请求 |
| --- | --- | --- | --- |
| 上 | 上 | 导航选择，自瞄开 | 开 / 开 |
| 上 | 中 | 导航选择，自瞄开 | 开 / 取决于连发状态 |
| 上 | 下 | 手动，自瞄开；小陀螺按切换记忆 | 关 / 连发状态可能保留 |
| 中 | 上 | 导航选择，自瞄关 | 开 / 开 |
| 中 | 中 | 导航选择，自瞄关 | 开 / 取决于连发状态 |
| 中 | 下 | 手动，自瞄关；小陀螺按切换记忆 | 关 / 连发状态可能保留 |
| 下 | 上 | 急停 | 关 / 关 |
| 下 | 中 | 急停 | 关 / 关 |
| 下 | 下 | 急停 | 关 / 关 |

右中 YAML 写着 `feeder.off`，解释器同时使用：

```cpp
fire_trigger = feeder_state_ || burst_mode_ || mouse_fire;
```

因此右下拨中切换 `burst_mode_` 后，即使右中配置为拨弹关闭，仍可能产生拨弹请求。右下的摩擦轮保持关闭。04-04 的版本将连发模式从发射请求条件中移出。

此表期间还有下列执行逻辑变化：

| 日期与提交 | 九种组合的执行差异 |
| --- | --- |
| 03-27 22:38 `feat: add navigation input handling and subscription to hub` | 四个“导航选择”组合正式接收导航速度；有效导航输入优先于 RC |
| 03-27 22:40 `refactor: remove action related` | Hub 去掉 Action 分支，保留导航与 RC 的选择 |
| 03-28 23:04 `fix: spin speed control` | 非急停期间可持续用拨轮预设旋转速度；组合选择沿用 |
| 03-29 02:46 `fix: set emergency_state to true by default in interpreters and make it possible to change the spin speed in emergency_state` | 初始状态采用急停，急停时也可预设旋转速度 |
| 03-29 03:05、03-30 18:17 的断连修正 | 断连急停处理完善；在线九种组合沿用 |
| 03-30 合入键鼠解析器 | 摇杆与键鼠计算重组，九种组合沿用 |
| 03-31 19:26 `feat: implement keyboard and mouse event handling with YAML config support` | 键鼠配置事件可额外改变状态；纯拨杆表沿用 |
| 03-31 21:52 `fix: correct cmd vel multiplier for nav` | 导航平移量换算修正，组合选择沿用 |
| 03-31 23:48 `feat: support auto_aim_switch and gimbal_scan_cmd from nav` | 四个导航组合的云台改为有效自瞄优先、否则扫描；导航侧自瞄检查直接看视觉输入，左中的自瞄标志仍为关 |
| 04-02 11:04 `fix: ignore stale gimbal scan command in hub` | 云台扫描检查消息新鲜程度；档位表沿用 |
| 04-02 14:11 `fix: Hub integration with behavior tree for no-referee mode` | 导航分支接入 `/cmd_spin` 状态；档位表沿用 |

03-27 22:38 以后，进入导航模式时解释器仍将 RC 的 `vx/vy/wz` 都置零。导航速度中断后，RC 旋转值为零。手动小陀螺则继续由左中拨上的切换状态决定。

### 4. 2026-04-04 02:42：右下强制关 RC 小陀螺

提交：`fix: pre 4.3`。

| 左拨杆 | 右拨杆 | 模式 | 摩擦轮 / 拨弹请求 |
| --- | --- | --- | --- |
| 上 | 上 | 导航，自瞄开 | 开 / 等待导航允许及有效自瞄 |
| 上 | 中 | 导航，自瞄开 | 开 / 关 |
| 上 | 下 | 手动，自瞄开，RC 小陀螺关 | 关 / 关 |
| 中 | 上 | 导航，左拨杆自瞄标志关 | 开 / 开 |
| 中 | 中 | 导航，左拨杆自瞄标志关 | 开 / 关 |
| 中 | 下 | 手动，RC 小陀螺关 | 关 / 关 |
| 下 | 上 | 急停 | 关 / 关 |
| 下 | 中 | 急停 | 关 / 关 |
| 下 | 下 | 急停 | 关 / 关 |

三项变化：

1. 右下档位新增 `spin_mode.off`。左中拨上的 toggle 执行顺序晚于档位动作，因此右下保持时，左中拨上当帧仍可能产生一次旋转请求，下一帧再次清零。
2. 发射请求改为 `feeder_state_ || mouse_fire`；连发状态只决定发射方式。
3. `navigation_enabled && autoaim_enabled` 的组合增加导航允许和视觉有效性检查。导航云台还要求 `/auto_aim_switch`；这一版 `is_autoaim_valid_nav()` 仍直接看视觉输入。

这一版 Hub 的 `/cmd_spin` 为非零时，还能在 RC 分支启动内部小陀螺。因此表中的“RC 小陀螺关”指拨杆状态；外部 `/cmd_spin` 单独参与实际输出。

### 5. 2026-09-15 与 2026-09-19 已提交版：右中控制 RC 小陀螺

09-15 提交：`chore: sep chassis controllers`。09-19 提交：`fix: 修正 NDJInterpreter 和 KeyboardMouseParser 的输入处理`。两版拨杆组合相同。

| 左拨杆 | 右拨杆 | 模式 | 摩擦轮 / 拨弹请求 |
| --- | --- | --- | --- |
| 上 | 上 | 导航，自瞄开；RC 旋转输出为 0 | 开 / 等待导航允许及有效自瞄 |
| 上 | 中 | 导航，自瞄开；RC 旋转输出为 0 | 关 / 关 |
| 上 | 下 | 手动，自瞄开，RC 小陀螺关 | 关 / 关 |
| 中 | 上 | 导航，自瞄关；RC 小陀螺关 | 开 / 开 |
| 中 | 中 | 手动平移，同时 RC 小陀螺开 | 关 / 关 |
| 中 | 下 | 手动，RC 小陀螺关 | 关 / 关 |
| 下 | 上 | 急停 | 关 / 关 |
| 下 | 中 | 急停 | 关 / 关 |
| 下 | 下 | 急停 | 关 / 关 |

关键代码关系：

- 左上开启导航许可；左中关闭导航许可。
- 右上再次开启导航许可；右中保留左拨杆给出的导航结果；右下关闭导航许可。
- 右中每帧置 `spin_mode=true`；右上、右下每帧置 `spin_mode=false`。
- 导航模式下，解释器将 `wz=0`。因此左上右中虽然保存了 `spin_mode=true`，导航超时转回 RC 后的旋转输出仍为 0。
- 左中右中允许摇杆平移并旋转。源码旧注释中的“仅小陀螺”适用于摇杆居中时。

09-15 还将底盘输出移到 `/chassis_command`；导航云台开始同时检查左拨杆自瞄许可。导航旋转由 `/cmd_vel.angular.z` 承接。RC 分支额外使用 `/cmd_spin` 时要求比赛已开始。

09-19 修正手动平移 `vy` 的符号及键鼠解析；拨杆动作定义和导航模式下的 `wz=0` 沿用 09-15 版本。

### 6. 当前工作区：左中拨上开启 RC 小陀螺

这是 09-19 提交之后的工作区改动，09-24 交接资料已经记载相关文件存在修改。Git 提交记录覆盖到 09-19；这部分具体编辑时间与作者待额外编辑记录补齐。

| 左拨杆 | 右拨杆 | 模式 | 摩擦轮 / 拨弹请求 |
| --- | --- | --- | --- |
| 上 | 上 | 导航，自瞄开；导航中断后按保存的 RC 小陀螺状态旋转 | 开 / 等待导航允许及有效自瞄 |
| 上 | 中 | 导航，自瞄开；导航中断后按保存的 RC 小陀螺状态旋转 | 关 / 关 |
| 上 | 下 | 手动，自瞄开；按保存的 RC 小陀螺状态旋转 | 关 / 关 |
| 中 | 上 | 导航，自瞄关；正常拨回左中后 RC 小陀螺关 | 开 / 开 |
| 中 | 中 | 手动；正常拨回左中后 RC 小陀螺关 | 关 / 关 |
| 中 | 下 | 手动；正常拨回左中后 RC 小陀螺关 | 关 / 关 |
| 下 | 上 | 急停 | 关 / 关 |
| 下 | 中 | 急停 | 关 / 关 |
| 下 | 下 | 急停 | 关 / 关 |

保存状态的具体规则：左中拨上设为开；左上回中、左中拨下设为关；键盘 V 可切换。开机直接处于左上时，首次消息尚无相邻拨动记录，内部初值为关。单看当前左上档位，旋转状态还需要结合此前拨动动作。

与 09-19 已提交版相比，当前工作区有两处关键变化：

```cpp
// 09-19: nav_mode_enabled_ 分支内
unified_output_.wz = 0.0;

// 当前: 放到导航/手动分支之后统一赋值
unified_output_.wz = spin_mode_enabled_ ? km_parser_.get_spin_speed() : 0.0;
```

同时，YAML 删除右上/中/下的固定小陀螺动作，加入左拨杆的开启/关闭动作。

实际结果：有效导航速度期间，Hub 使用 `/cmd_vel.angular.z`，其中可包含行为树 `/cmd_spin`。导航速度间隔达到 0.2 秒后，Hub 转回 RC；左中拨上曾开启的旋转状态随即参与输出。该变化解释了“左上右中，导航走动时旋转停，导航更新间断时又旋转”的现象。

## 二、前身 Python 控制器

仓库：`src/sentry_controller`。这是另一套启动入口，历史表与 C++ 控制器分开阅读。

### 2026-03-20 14:08 至 03-21 02:45 前

| 左拨杆 / 右拨杆 | 右上 | 右中 | 右下 |
| --- | --- | --- | --- |
| 左上 | 手动，进入左上切换旋转 | 手动，进入左上切换旋转 | 手动，进入左上切换旋转 |
| 左中 | 手动，旋转按切换记忆 | 手动，旋转按切换记忆 | 手动，旋转按切换记忆 |
| 左下 | RC 发布零速度 | RC 发布零速度 | RC 发布零速度 |

右拨杆尚未参与底盘模式选择；外部自瞄许可控制云台。03-21 01:04 的转向保持改动沿用此表。

### 2026-03-21 02:45 与 2026-03-23 19:00

| 左拨杆 / 右拨杆 | 右上 | 右中 | 右下 |
| --- | --- | --- | --- |
| 左上 | 导航通道，云台自瞄 | 导航通道，云台导航姿态 | 手动，旋转按切换记忆 |
| 左中 | 导航通道，云台自瞄 | 导航通道，云台导航姿态 | 手动，旋转按切换记忆 |
| 左下 | RC 发布零速度、云台停 | RC 发布零速度、云台停 | RC 发布零速度、云台停 |

03-21：右上/中时，RC 解释器持续向与导航共用的话题发零速度，实际底盘速度取决于最新到达的消息。右上为自瞄发射模式，右中开摩擦轮并通过下拨中动作切换供弹方式；右下关闭摩擦轮。火控节点有独立启用参数；03-21 10:48 的配置曾关闭该节点。

03-23：进入导航档时改为仅清零一次；火控改为比赛开始后右上/中开摩擦轮，右上同时结合视觉发射条件。拨杆组合表沿用。该阶段火控主要判断右拨杆和比赛状态，左下停车由 RC/云台各自处理。

### 2026-03-24 00:14、00:16：分开导航和 RC 速度话题

| 左拨杆 / 右拨杆 | 右上 | 右中 | 右下 |
| --- | --- | --- | --- |
| 左上 | 导航速度，云台自瞄 | 导航速度，云台导航姿态 | 手动，旋转按切换记忆 |
| 左中 | 导航速度，云台自瞄 | 导航速度，云台导航姿态 | 手动，旋转按切换记忆 |
| 左下 | 底盘仍选择导航话题；云台停 | 底盘仍选择导航话题；云台停 | RC 零速度，云台停 |

解释器把左下的零速度发到 `/cmd_vel_rc`，底盘却只看右拨杆选择 `/cmd_vel` 或 `/cmd_vel_rc`。因此这个历史实现的左下右上/中仍可能执行导航速度。00:16 补充导航平移单位换算，组合选择沿用。火控仍采用 03-23 的比赛条件。

### 2026-03-24 01:00 至 03-30

| 左拨杆 / 右拨杆 | 右上 | 右中 | 右下 |
| --- | --- | --- | --- |
| 左上 | 导航，自瞄许可开 | 导航，自瞄许可开 | 手动，自瞄许可开，旋转按切换记忆 |
| 左中 | 导航，自瞄许可关 | 导航，自瞄许可关 | 手动，自瞄许可关，旋转按切换记忆 |
| 左下 | 底盘仍选择导航话题；云台停 | 底盘仍选择导航话题；云台停 | RC 零速度，云台停 |

这个版本把云台自瞄许可移到左上；右上/中负责底盘导航选择。小陀螺调速改用拨轮，开启旋转时重置为默认速度。03-27 增加导航云台巡航；03-30 更新裁判话题；九种组合沿用。

### 2026-09-15 15:52：Python 仓库最后一版

九种组合沿用上表。差异为：

- 导航速度超过 0.5 秒未更新时，底盘速度清零。
- 左中加右上/中可直接开启摩擦轮；左上加右上/中仍要求比赛开始。
- 左下使该版火控停止；底盘右上/中仍按导航消息是否新鲜选择速度，云台由左下停止。

上述 Python 历史的左下处理与当前 C++ Hub 的统一急停存在实际差异。当前 NDJ 为主输入时，C++ Hub 先执行急停判断，再选择导航/RC。

## 三、保存状态和复核位置

检查了 04-02、04-04 的修改前保存记录，对应 YAML、NDJ 解释器和 Hub 的有效内容与最终提交一致。03-30 保存的临时工作状态采用同一拨杆配置，导航 `wz=0` 和连发触发条件也与当时主线一致，故对应第一部分第 3 张组合表。

03-30 合入的分支还包含 `feat: implement unified keyboard and mouse input parser`、`feat: support spin mode and speed attributes`、格式调整和底盘功率相关改动。它们沿用第 3 张拨杆组合表；键鼠同时输入时按各版解析器处理。前身 Python 仓库 03-20 13:41 的初始目录为 `infantry_controller`，哨兵 NDJ 解释器于同日 14:08 加入，因此第二部分从哨兵版本开始列出。

最近两个 C++ 已提交版本：09-15 改档位定义，09-19 改输入方向；当前未提交修改再改小陀螺来源。最近的导航里程计修复沿用查询开始时的拨杆配置。

当前源码位置：

- `src/universal_controller/config/ndj_definition.sentry.yaml:15`：两侧档位动作。
- `src/universal_controller/config/ndj_definition.sentry.yaml:33`：左拨杆切换动作。
- `src/universal_controller/src/interpreters/ndj_interpreter.cpp:278`：导航模式与 RC 速度。
- `src/universal_controller/src/interpreters/ndj_interpreter.cpp:328`：先左档、后右档、再切换动作。
- `src/universal_controller/src/hub/hub_arbitration.cpp:41`：急停、导航和 RC 选择；`:131`：旋转来源；`:235`：发射条件。

查询时 C++ HEAD 对应 2026-09-19 12:41 提交。相对这个版本，`HEAD~1` 为 09-15，`HEAD~2` 为 04-04，`HEAD~44` 为 03-26 首版配置，`HEAD~45` 为 03-25 首版 YAML。可在该仓库用 `git show 'HEAD~1:config/ndj_definition.sentry.yaml'` 查看当时原文。

## 四、C++ 主线提交与组合表对应

以下按祖先关系从早到晚排列。组合表编号指第一部分；同表的具体执行变化见对应章节。日期为提交中保存的本地时间。

| 时间 | 提交说明 | 组合表 |
| --- | --- | --- |
| 2026-03-23 01:42:12 | Initial commit | 结构及文档阶段 |
| 2026-03-25 00:10:07 | Merge pull request #2 from oxoxox-oxox/main | 结构及文档阶段 |
| 2026-03-25 01:03:05 | feat: init structure | 结构及文档阶段 |
| 2026-03-25 01:33:22 | docs: update plan | 结构及文档阶段 |
| 2026-03-25 02:31:16 | feat: init proj | 旧字段草稿，见表 1 说明 |
| 2026-03-25 02:32:05 | refactor: update NDJ and VTM interpreters to utilize InputProcessor | 旧字段草稿，见表 1 说明 |
| 2026-03-25 02:43:03 | refactor: use universal_controller | 旧字段草稿，见表 1 说明 |
| 2026-03-25 02:58:08 | docs: update build conf and readme | 旧字段草稿，见表 1 说明 |
| 2026-03-25 03:04:38 | fix: interpreter entry linking and msg spec | 表 1 |
| 2026-03-25 03:19:11 | refactor: unify parameter structure and update topic references in configuration files | 表 1 |
| 2026-03-25 19:09:43 | refactor: update topic parameters in configuration and improve parameter declaration checks | 表 1 |
| 2026-03-25 20:00:07 | refactor: consolidate interpreter nodes, update related configurations | 表 1 |
| 2026-03-25 20:42:26 | feat: add unified input timeout handling and improve RC interpreter state management | 表 1 |
| 2026-03-25 23:22:12 | feat: add configurable ndj_definition | 表 2 |
| 2026-03-26 16:06:56 | feat: add vtm ctrl, update schema | 表 3 |
| 2026-03-26 22:37:09 | refactor: rename trigger definitions to switch for clarity | 表 3 |
| 2026-03-27 02:01:04 | fix: update controller parameters and improve friction control logic | 表 3 |
| 2026-03-27 02:01:24 | feat: add yaw_center_ecd parameter to chassis configuration | 表 3 |
| 2026-03-27 21:39:30 | fix: adjust shot period and update motor enable logic in fire controller | 表 3 |
| 2026-03-27 21:59:47 | fix: ctrl source slug | 表 3 |
| 2026-03-27 22:01:34 | refactor: separate hub cod | 表 3 |
| 2026-03-27 22:08:39 | refactor: integrate rc hub into hub_rc | 表 3 |
| 2026-03-27 22:09:19 | chore: update gitignore | 表 3 |
| 2026-03-27 22:38:54 | feat: add navigation input handling and subscription to hub | 表 3 |
| 2026-03-27 22:40:30 | refactor: remove action related | 表 3 |
| 2026-03-27 23:14:53 | refactor: update referee cb func name | 表 3 |
| 2026-03-28 02:01:46 | fix: dji_referee_protocol/msg use ros2 msg ver | 表 3 |
| 2026-03-28 21:56:55 | fix: adjust pitch and yaw center ecd, update topic routes | 表 3 |
| 2026-03-28 21:57:30 | fix: revert pitch adjustment | 表 3 |
| 2026-03-28 21:59:45 | fix: gear interpreter | 表 3 |
| 2026-03-28 22:29:33 | fix: btn and trigger interpreter | 表 3 |
| 2026-03-28 23:04:35 | fix: spin speed control | 表 3 |
| 2026-03-28 22:07:47 | docs: fix typo | 表 3 |
| 2026-03-29 00:26:36 | fix: update motor commands in stop_all and handle emergency stop in hub_core | 表 3 |
| 2026-03-29 01:29:15 | feat: add left_joystick_x handling | 表 3 |
| 2026-03-29 01:29:45 | fix: yaw rotation direction | 表 3 |
| 2026-03-29 02:46:16 | fix: set emergency_state to true by default in interpreters and make it possible to change the spin speed in emergency_state | 表 3 |
| 2026-03-29 03:05:11 | fix: handle emergency state in publish_unified when not connected 解析遥控器发来的 online 字段 到 connected Fixes #4 | 表 3 |
| 2026-03-29 03:57:12 | fix: decrease dial sensitivity | 表 3 |
| 2026-03-29 14:27:34 | refactor: handle autoaim in hub | 表 3 |
| 2026-03-30 14:10:42 | Merge pull request #11 from AIM-EC:feature/chassis-power-limit | 表 3 |
| 2026-03-30 16:18:05 | fix: update referee topics in configuration files | 表 3 |
| 2026-03-30 18:17:57 | fix: emergency stop when rc disconnected | 表 3 |
| 2026-03-31 02:53:40 | style: define clang format, adjust code formatting | 表 3 |
| 2026-03-31 19:26:00 | feat: implement keyboard and mouse event handling with YAML config support | 表 3 |
| 2026-03-31 21:49:44 | feat: enhance km func def, add on_released_after_s support | 表 3 |
| 2026-03-31 21:52:30 | fix: correct cmd vel multiplier for nav | 表 3 |
| 2026-03-31 23:48:27 | feat: support auto_aim_switch and gimbal_scan_cmd from nav | 表 3 |
| 2026-04-01 00:35:51 | feat: power limit for infantry | 表 3 |
| 2026-04-01 03:03:20 | feat: super cap writeback | 表 3 |
| 2026-04-01 07:43:33 | feat: keep pub supercap topic on emergency stop | 表 3 |
| 2026-04-02 02:18:28 | feat: sentry supercap support | 表 3 |
| 2026-04-02 02:20:31 | feat: sentry EC supercap charging | 表 3 |
| 2026-04-02 03:30:33 | fix: add missing supercap msg header | 表 3 |
| 2026-04-02 11:04:45 | fix: ignore stale gimbal scan command in hub | 表 3 |
| 2026-04-02 14:11:09 | fix: Hub integration with behavior tree for no-referee mode | 表 3 |
| 2026-04-04 02:42:44 | fix: pre 4.3 | 表 4 |
| 2026-09-15 15:53:25 | chore: sep chassis controllers | 表 5 |
| 2026-09-19 12:41:42 | fix: 修正 NDJInterpreter 和 KeyboardMouseParser 的输入处理 | 表 5 |
| 当前工作区 | 左拨杆开启/关闭小陀螺，导航分支保留 RC 旋转输出 | 表 6 |
