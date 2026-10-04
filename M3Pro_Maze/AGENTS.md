# M3Pro 迷宫自主探索与方块收集系统 — 项目指南

本项目为 ROSMASTER M3 Pro 小车开发二维迷宫自主探索 + 彩色方块收集系统。
**编码前必读**：本文件记录的是已在本机逐条验证过的硬件接口事实，不要凭通用 ROS 经验臆测话题名或单位。

---

## 0. 文风要求（适用于本文件、`PLAN.md` 及所有代码注释）

- **简洁明了**：一句话一个事实；能用表格不用段落；删除修饰语、复述与"综上所述"类套话。
- **严谨科学**：数值必带单位与来源；结论必带依据（文件路径:行号、命令输出、规则原文）；区分「已验证 / 待验证 / 推测」，推测不得写成事实。
- **内容必须经过验证**：硬件接口、话题名、单位、量程等只允许写入本机实测确认的结果；未经验证的内容进入文末「待验证清单」，不混入正文。
- 新增或修改事实条目时，同步注明验证方式（命令 / 文件）与日期。

---

## 1. 任务定义

- 场地 **2.8m × 2.8m = 7 格 × 7 格**，通道宽 **40cm**，挡板高 55cm，通道正中贴**黑色引导线**
- 车体 **21.5cm(宽) × 27.5cm(长)** → 通道内**单侧余量仅 9.25cm**，循迹精度需优于 ±5cm
- 迷宫为**树形结构**，一入口一出口；红/绿/黄/蓝方块**各 2 个（共 8 个）**，分布在不同深度
- 计分 = 实际完成时间 + 漏取罚时（按**深度与颜色**，系数赛前未知、因组而异，与小车基准速度挂钩）
- **硬约束**：附加装置任何状态（收纳/展开/工作）横向宽度 ≤ 21.5cm；含机械臂总高 ≤ 55cm；不得损毁迷宫
  - 推论：任何收集机构**只能覆盖 40cm 通道的中央 21.5cm**（规则"横跨通道的扫杆"与宽度上限并存，按宽度上限取严）
- 方块由本队自备（仅核验颜色与数量），**不允许贴 AprilTag 等标记**（已确认），识别只能走颜色路线

### 已确定的方案选型
| 维度 | 决策 |
|---|---|
| 收集方式 | **机械臂夹爪逐块抓取**（零硬件改动），不引入被动铲/扫杆 |
| 巡线传感器 | **复用现有 Orbbec 相机**，机械臂固定"看地"姿态；激光走廊居中作冗余 |
| 比赛策略 | **8 块全收集**，顺序"先深后浅"（时间不足时漏的是浅层低罚时块） |
| 出口处理 | 方块**随车携带即可**，出口处不卸货 → 收集筐为开口托盘 |
| 方块标记 | **不允许**，只能颜色 + 深度识别 |

---

## 2. 运行环境

- NVIDIA Jetson Orin，Ubuntu 22.04.5，L4T **R36.4.4**（JetPack 6.x），aarch64
- **ROS 2 Humble**，`ROS_DOMAIN_ID=30`（**必须**，否则看不到任何话题）
- 两个工作区：`/home/jetson/yahboomcar_ws`（相机/机械臂/视觉）、`/home/jetson/M3Pro_ws`（雷达/SLAM/导航/EKF）
- 项目代码包：`/home/jetson/yahboomcar_ws/src/maze_explorer`

### 启动顺序（严格按序）
```bash
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ws/install/setup.bash
source /home/jetson/M3Pro_ws/install/setup.bash

sh /home/jetson/start_agent.sh                              # 1. 下位机 micro-ROS（雷达/IMU/里程计）
ros2 launch slam_mapping bringup.launch.py                  # 2. 雷达链 + IMU 滤波 + EKF
ros2 launch orbbec_camera dabai_dcw2.launch.py              # 3. 深度相机
ros2 launch maze_explorer maze_bringup.launch.py            # 4. 自研系统
```

---

## 3. 硬件接口速查（已验证）

### 底盘（麦轮全向，STM32 下位机经 micro-ROS 串口）
| 项 | 值 |
|---|---|
| 控制话题 | **`/cmd_vel`**，`geometry_msgs/Twist` |
| 有效字段 | `linear.x`(m/s)、`linear.y`(m/s，**支持横移**)、`angular.z`(rad/s) |
| 里程计 | **`/odom_raw`**（STM32 直出，高频，**闭环用这个**）；`/odom` 为 EKF 输出但仅 **6Hz**、位置靠积分**长期漂移** |
| 坐标系 | `frame_id="odom"`，`child_frame_id="base_footprint"` |
| 标定系数 | `odom_angular_scale_correction=0.75`、`odom_linear_scale_correction=1.0` |
| 建议速度 | 迷宫内 **0.1~0.25 m/s**、**0.3~0.6 rad/s**（防麦轮打滑） |
| 下位机节点 | **`/YB_Node`**（STM32 固件经 micro-ROS 暴露）。订阅 `/cmd_vel`、`/arm6_joints`、`/arm_joint`、`/beep`、`/rgb`；发布 `/odom_raw`、`/scan0`、`/scan1`、`/imu/data_raw`、`/battery`(`std_msgs/Float32`，实测 12.3，量纲未核验) → **底盘与机械臂都无需上位机驱动节点**，agent 在线即可；`/arm6_joints` 的订阅者就是它 |

> ⚠️ `/cmd_vel` **无超时停车保护**，须 ≥10Hz 持续发布，停止时**显式发全 0 Twist**。
>
> **验证记录（2026-10-04）**：`ros2 node info /YB_Node` 实测上述接口；
> `/cmd_vel` 发布者计数实测为 2（`joy_ctrl` 与自研 `base_driver` 并存时）→
> **跑任务前必须关手柄自启**，否则两个发布者互相覆盖。

### 激光雷达
| 项 | 值 |
|---|---|
| 数据链 | `/scan0`+`/scan1` → merger → `/scan_multi` → filter(<0.18m 置 inf) → **`/scan`** |
| 订阅 | **`/scan`**，`sensor_msgs/LaserScan`，`frame_id="base_link"` |
| 参数 | 360°、角分辨率 1°、量程 **0.05~4.0m** |

### 深度相机（Orbbec DaBai DCW2）
| 项 | 值 |
|---|---|
| 彩色 | `/camera/color/image_raw` 640×480 **@30fps** |
| 深度 | `/camera/depth/image_raw` 640×400 **@10fps**，`32FC1`，**单位 mm** |
| 配准 | `depth_registration` 默认 **false** → 彩色/深度**未对齐**，需 `ApproximateTimeSynchronizer` + 内参映射 |
| 内参 | K = [477.57, 0, 319.38; 0, 477.56, 238.64; 0,0,1] |
| 安装 | ⚠️ **相机本体装在机械臂第 4 连杆**（`DCW2_Joint` → parent `arm4`） |

> 🔑 **核心约束**：机械臂一动，相机视角就变。因此必须在**巡线姿态下完成方块定位**后再让机械臂抓取。

### 机械臂（6-DOF DOFBOT 系，舵机 0~180°）
| 项 | 值 |
|---|---|
| 六关节下发 | **`arm6_joints`**，`arm_msgs/ArmJoints`（`int16 joint1..joint6` + `int16 time`(ms)，demo 用 2000） |
| 单关节 | `arm_joint`，`arm_msgs/ArmJoint`（`id`,`joint`,`time`） |
| 逆/正解 | 服务 **`get_kinemarics`**，`arm_interface/ArmKinemarics`（`kin_name`="ik"/"fk"） |
| 夹爪 | **joint6：`30`=张开，`120~140`=夹紧**（须按方块尺寸实测标定） |
| 抓取包络 | 水平距离 **0.13~0.25m**（<130mm 判无效；demo 对准 200~220mm）→ **对位目标 0.20m** |
| 初始位姿 | `[90, 150, 12, 20, 90, 0]`；`follow_line.py` 用 `[90, 90, 12, 20, 90, 0]` |
| 角度映射 | `(joint - 90) * DE2RA`；`tool_param = 0.12`（夹爪长 12cm） |
| **到位反馈** | **无**：`Curjoints` 仅为 demo 节点回显**指令值**，`arm_driver` 无反馈 publisher，`get_kinemarics`(fk) 也是重算指令 → 到位判定只能用 `time` 计时契约 + 视觉复核 |

> **验证记录**：本节话题/单位为此前 `ros2 topic list|hz|echo` 实测（日期未留存）；`到位反馈=无`、`grasp.py` 触发源、掉头几何余量（34.9cm，**计算值**非实测）为 2026-09-27 源码 grep / 几何计算确认。

---

## 4. 可复用的现有代码（**不要重复造轮子**）

| 用途 | 路径 |
|---|---|
| 颜色识别库 `color_detect` | `/home/jetson/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/color_common.py` |
| 方块识别+对准 demo | `.../M3Pro_demo/color_recognize.py` |
| 抓取 demo（`GraspNode`） | `.../M3Pro_demo/grasp.py` |
| 巡线 demo（PID + HSV） | `.../M3Pro_demo/follow_line.py` |
| 夹爪朝向计算 | `.../M3Pro_demo/compute_joint5.py` |
| 闭环走直线（tf 反馈模板） | `/home/jetson/M3Pro_ws/src/calibration/calibration/calibrate_linear.py` |
| 闭环转角度（tf 反馈模板） | `/home/jetson/M3Pro_ws/src/calibration/calibration/calibrate_angular.py` |
| 开环运动封装 | `/home/jetson/M3Pro_ws/src/largemodel/largemodel/action_service.py:821-937` |

### `color_common.py` 关键函数
```python
object_follow(img, hsv_msg)       # 单目标 → (Center_x, Center_y, Center_r)
object_follow_list(img, hsv_msg)  # ★多目标 → 中心列表（同屏多方块用这个）
object_shape(img, hsv_msg)        # → corners(4×2)，配合 compute_joint5 算夹爪朝向
Roi_hsv(img, Roi)                 # 鼠标框选学习 HSV
read_HSV(path) / write_HSV(path, value)
simplePID.compute(target, current) / .reset()
```

### 现有 HSV 阈值（`.../M3Pro_demo/*.text`，格式 `Hmin,Smin,Vmin,Hmax,Smax,Vmax`）
| 颜色 | 值 |
|---|---|
| 红 | `0, 154, 107, 184, 253, 255` |
| 绿 | `34, 130, 205, 125, 253, 255` |
| 蓝 | `55, 196, 137, 125, 253, 255` |
| 黄 | `23, 99, 235, 125, 253, 255` |
| 巡线 | `34, 98, 74, 125, 253, 255` ⚠️ **可疑** |

---

## 5. 关键陷阱清单

1. **`ROS_DOMAIN_ID=30`** 必须设置，且要 source **两个**工作区的 `install/setup.bash`
   - 衍生陷阱：`/cmd_vel` **无超时停车保护**，下位机锁存最后一条指令。代码侧已由
     `CommandWatchdog`（独立线程，`cmd_watchdog_timeout=0.5s`）覆盖"主线程/executor
     卡死"，但**进程被 SIGKILL/OOM 时线程一并消失** → 该场景只能物理断电（见 `SAFETY.md`）
2. **`/home/jetson/joy_control/joy.sh` 开机自启**会跑 `yahboom_joy_M3Pro` 并发 `JoyState`(Bool)；部分节点在 `JoyState=True` 时会主动发零速 → **运行自研程序前先关掉手柄自启**
3. **`LineFollowHSV.text` 的 `V_min=74` 偏高**，不符合黑色电工胶布的低明度特征 → **必须用 `Roi_hsv` 现场重标黑线阈值**；四色方块阈值同理按自备材质重标
4. **相机在机械臂上** → 巡线姿态下完成定位，抓取期间相机随臂移动属"盲抓"
5. **深度图仅 10fps 且未配准** → 必须时间同步 + 内参映射
6. **`grasp.py` 用 `time.sleep(2.5)` 硬编码**，连续作业需改为状态机/可配置时长
7. **里程计位置靠积分漂移** → 每格到位用"侧墙对称度 + 开口宽度 + 里程计进距"三取二验证（路口无前墙，不能只靠前墙）
8. 相机 SDK 每秒刷 `Property is not supported! propertyId: 1003` 警告（无害），自研日志**禁止每帧打印**以免雪上加霜
9. **`grasp.py` 的触发源是 `/PosInfo`（AprilTag）**，而比赛方块不许贴标记 → 该链路的"识别→抓取"部分不可直接复用，只复用其 `pixel_to_camera_depth` + `EndToCamMat` 解算与 IK 调用路径
10. **机械臂无关节反馈**（见 §3 到位反馈行）→ 禁止写"等待到位"式阻塞，改用 `arm6_joints.time` 计时 + 抓后视觉复查
11. **原地 180° 掉头余量仅 ~2.5cm/侧**（车对角 34.9cm vs 通道 40cm）→ 掉头优先安排在路口，全程激光近距守卫
12. **Orbbec 相机在 USB2.0 下会自行掉线重连**：实测日志 `onDeviceDisconnected`（启动约 27s 后）→ 约 7s 后 `Connecting to the default device` 重连。断连窗口内 `maze_run` 自检会**误报"未收到相机图像"**，与配置无关 → 等 10s 重试；根治要换 USB3.0 口/直连不 Hub，或减少同时开启的流（当前 color+depth+ir 三路都开）
    - 依据：2026-10-04 `~/maze_test_logs/*/camera_ik.log`（`Start device cost 867ms` → 27s 后 `onDeviceDisconnected` → 6.2s 后重连）
13. **一键脚本** `M3Pro_Maze/scripts/maze_test.sh`（文档仓，非 ROS 包）：默认只起依赖链 + 四项自检、**不动车**；`--run` 才驱动底盘且要打字确认。它用 `setsid` 给每条依赖链独立进程组，`--stop` 对整个组发 SIGINT（只杀 launch 父进程会留下孤儿节点，实测踩过）

---

## 6. 编码约定

### 6.1 代码风格

- 基线：**PEP 8**，`ament_flake8` + `ament_pep257` 必须过检（本机已装）；4 空格缩进，无 Tab
- 命名：函数/变量 `snake_case`，类 `CamelCase`，常量 `UPPER_SNAKE`，模块名即节点名（`base_driver.py` → 节点 `base_driver`）
- 文件头与现有 demo 一致：`#!/usr/bin/env python3` + `# -*- coding: utf-8 -*-`（仅脚本类入口需要）
- 导入顺序：标准库 → 第三方(cv2/numpy/yaml) → ROS(rclpy/*_msgs) → 本包模块，每组内按字母序、组间空行分隔
- 每个模块与 `class`、**公有方法**写 docstring（一句话概括 + 输入/输出）；私有函数不强制
- 公有函数标注类型签名（`def pick_target(blocks: list[BlockTarget]) -> BlockTarget:`），不引入 mypy 依赖
- 注释与日志用中文，遵守 §0 文风：一句一事实、数值带单位
- 禁止：行尾多余空格、无用 import、`except:` 裸捕获、模块级可执行副作用（可执行逻辑进 `main()`）

### 6.2 工程与 ROS 节点约定

- **零侵入**：全部新增代码放 `maze_explorer` 包，**不修改任何现有包源码**
- 控制环 **20Hz**（`create_timer(0.05)`）；闭环订阅 `/odom_raw`
- 日志：状态迁移记 INFO、异常记 WARN，**禁止每帧打印**
- 参数全部外置到 `config/*.yaml`，**禁止硬编码**（相机内参除外，可沿用已标定值）
- 所有 `time.sleep` 驱动的机械臂动作改为**状态机 + 超时判定**，便于中断与压缩节拍
- 失败处理：抓取失败自动重试 1 次，二次失败登记为待办继续探索，**不阻塞整体流程**

---

## 7. 目录结构

```
/home/jetson/yahboomcar_ws/src/maze_explorer/
├── package.xml / setup.py
├── config/           maze_params.yaml（场地/速度/激光扇区）
│                     hsv_params.yaml（黑线+四色阈值）
│                     arm_poses.yaml（巡线/抓取/放置位姿、包络）
├── launch/           maze_bringup.launch.py
├── maze_explorer/    base_driver / sensors / line_detector / block_detector
│                     grid_mapper / dfs_planner / motion_controller
│                     arm_controller / grasp_fsm / mission_manager
│                     score_logger / calibration_tool
└── scripts/          run_maze.sh（含 rosbag 录制）
```

文档：`AGENTS.md`（本文件）、`PLAN.md`（实施方案）、`ARCHITECTURE.md`（架构与依赖）、
`SAFETY.md`（安全审计与上机检查清单）、`README.md`（入门）

---

## 8. 待验证清单（未实测，禁止当事实使用）

| 项 | 需验证方式 |
|---|---|
| 雷达扫描平面高度、能否扫到地面方块 | 实测：方块置于雷达前读 `/scan` |
| 通道内原地 180° 掉头实际余量 | 实测：慢速旋转 + 近距激光守卫 |
| 自备方块尺寸/重量对夹爪与包络的影响 | 实测标定 `joint6` 开合角 |
| 出口判定区域的物理形式与识别方式 | 赛前问组委会/现场观察 |
| 罚时系数表（深度 × 颜色 × 基准速度） | 赛前获取 |
| 走廊居中（激光）精度能否优于 ±5cm | 实测跑直线统计横向偏差 |

---

## 9. 版本管理（双仓库，各自独立）

本项目用**两个独立 git 仓库**分别管理文档与代码，互不隶属：

| 仓库 | 路径 | 管理内容 |
|---|---|---|
| **文档仓库** | `/home/jetson/Desktop`（`.git` 位于 Desktop 根） | `M3Pro_Maze/` 下的 `README.md`、`AGENTS.md`、`PLAN.md`、`ARCHITECTURE.md`、`SAFETY.md`，以及根 `.gitignore` |
| **代码仓库** | `/home/jetson/yahboomcar_ws/src/maze_explorer` | 自研 ROS 2 包源码（本项目的全部代码） |

### 为什么分成两个仓库

ROS 2 包必须位于 colcon 工作区的 `src/` 下，才能被 `colcon build` 发现并构建。若把代码搬进 Desktop 文档仓库，就必须改用 `colcon build --base-paths` 并将 `build/`、`install/`、`log/` 一并搬过去，导致文档仓库混入 MB 级构建产物。而文档与代码的改动节奏、评审方式本就不同，**分仓更清晰**。

曾评估过合并方案（代码放 Desktop 仓库 + `--base-paths` 构建），已否决，按双仓库执行。

### 不纳入版本管理（两个仓库都要忽略）

- **厂商/第三方代码**：`OrbbecSDK_ROS2`、`yahboom_yolov8`、`M3Pro_demo`、`M3Pro_core`、`yahboom_M3Pro_laser` 等——非本项目产出，且体积巨大，**绝不入库**
- **ROS 构建产物**：`build/`、`install/`、`log/`
- **Orbbec SDK 运行日志**：`Log/`、`*.log`（每次运行都会变化）
- **Python 缓存**：`__pycache__/`、`*.pyc`

### 代码仓库初始化

在阶段 1 创建包骨架时执行：

```bash
cd /home/jetson/yahboomcar_ws/src/maze_explorer
git init
# 写入 .gitignore：build/ install/ log/ __pycache__/ *.pyc *.log
git add -A && git commit -m "..."
```

⚠️ 只对 `maze_explorer` 目录建仓库，**不要**对整个 `yahboomcar_ws` 执行 `git init`（会把全部厂商代码和构建产物卷进去）。

### 提交约定

- **小步提交**：每完成一个可验证步骤（写码 / 单测通过 / 实机验证通过 / 文档同步）即提交，不攒大提交
- 模块完成的提交，信息中标注模块名与验收结果
- **commit message 格式**：`<type>(<scope>): <中文简述>`，type 取自下表，scope 为模块名（如 `base_driver`），简述 ≤50 字、用祈使句

| type | 用途 |
|---|---|
| `feat` | 新增功能/模块 |
| `fix` | 修 bug |
| `test` | 新增/修改测试 |
| `docs` | 仅文档（AGENTS/PLAN/README） |
| `refactor` | 重构，不改行为 |
| `chore` | 构建、依赖、配置、gitignore |
| `perf` | 性能优化 |
| `verify` | 实机验证记录/参数标定结果 |

示例：
```
feat(line_detector): 黑线中心提取与压线遮挡状态
test(dfs_planner): 后序遍历以出口为终点的单测
verify(arm): joint6 开合角标定 30/135，单块节拍 7.4s
```
- 正文（可选）写"改了什么 + 依据/验收数据"；两仓各自独立提交，互不影响
- 两仓均**只做本地提交**，不配置 remote、不 push（除非明确要求）

---

## 10. 实现状态与两个入口（V2）

### 10.1 实现状态

代码仓库 `/home/jetson/yahboomcar_ws/src/maze_explorer` 已完成 V1 全量实现、V2 缺陷修复与 V3 安全保护。当前：**pytest 91/91 通过**、`colcon build` 通过、无硬件启动烟测通过。

| 层 | 模块 | 状态 |
|---|---|---|
| 硬件抽象 | `base_driver` / `sensors` / `arm_controller` | 完成（含航向源开关 `yaw_source`、速度看门狗 `CommandWatchdog`） |
| 感知 | `line_detector` / `block_detector` | 完成 |
| 建图决策 | `grid_mapper` / `dfs_planner` | 完成 |
| 控制 | `motion_controller` / `grasp_fsm` | 完成（含传感器保鲜门禁） |
| 编排 | `mission_manager` | 完成（INIT/EXPLORE/APPROACH/GRASP/RETURN/FINISH/FAULT） |
| 入口与工具 | `run_entry` / `calib_entry` / `calibration_tool` | 完成 |
| 安全 | 看门狗 + 激光/相机保鲜度 + 失效即停 | 完成（V3，见 §10.5 与 `SAFETY.md`） |
| 测试 | `test_perception` / `test_grid_mapper` / `test_dfs_planner` / `test_motion_scaling` / `test_approach` / `test_safety` | 91 项 |

说明：`PLAN.md` 中的 `DETECT` 状态已合入 `EXPLORE`（检测与探索在同一状态内），属实现简化、非缺陷；`score_logger` 尚未实现（如需记录成绩时间可后续补）。

### 10.2 两个用户入口

```bash
# 入口 A：一键探索（先逐项自检，通过后自动开跑）
ros2 run maze_explorer maze_run
ros2 run maze_explorer maze_run --check-only    # 只自检不运动，首次务必先跑

# 入口 B：现场标定（无参数进菜单，也可直达）
ros2 run maze_explorer maze_calib
ros2 run maze_explorer maze_calib hsv           # 或 line_pose / motion
```

其余 6 个入口（`mission_manager`、`calibration_tool`、`line_detector`、`block_detector`、`grid_mapper`、`arm_controller`）保留作调试用途。

### 10.3 V2 修复的关键缺陷（防止踩回同一个坑）

| 编号 | 问题 | 后果 |
|---|---|---|
| B1 | 运动原语内绕过注入的 spin 回调，直呼 `rclpy.spin_once(self._node, ...)` | odom 回调不执行 → 转向闭环失效、原地转到超时 |
| B2 | 转向缩放用乘法（应为除法：真实转角 = 系数 × 读数 ⇒ 目标 = 期望 / 系数） | 90° 只转 67.5°，逐格累积撞墙 |
| B3 | 远处方块立即对位，且对位仅 4 步 × 0.15m | 脱离格心打乱 DFS 拓扑；>0.8m 方块必然抓不到 |
| B11 | `spin_until_future_complete(self, ...)` 与外部 executor 冲突 | 首次 IK 调用抛异常，抓取完全不可用 |

其余 B4~B10 见代码提交记录。**B1/B2/B3 的语义已被回归测试锁定**（`test_motion_scaling.py` / `test_approach.py`）——改动相关逻辑后若这两个文件失败，说明踩回了旧坑。

### 10.4 现场复验清单（V2 之后必须重做）

| 项 | 为什么必须重做 |
|---|---|
| 黑线/四色 HSV | 现场光照与方块材质每次都不同 |
| `odom_angular_scale_correction` | **默认已改为 1.0（不修正）**，必须用 `maze_calib motion` 实测后填写；若改用 `yaw_source:=odom` 需重新标 |
| 机械臂巡线姿态 `line_pose` | 决定相机视角，装车后必标 |
| 相机安装外参 `mount_xyz / mount_rpy` | 标定后可置 `mount_calibrated:=true`，否则抓取只用相机系近似 |
| 车载收集筐与 `place_pose` | 收集筐固定方式与位置决定放置位姿 |
| `detect_trigger_distance`（默认 0.60m） | 依相机实际视野与巡线姿态调整 |

### 10.5 V3 安全保护（2026-10-03 实现并验证）

背景：本机 `/cmd_vel` **无超时停车保护**，下位机锁存最后一条指令；而发布依赖单线程
executor 里主线程的 spin。完整审计与残留风险见 `SAFETY.md`。

| 机制 | 位置 | 语义 |
|---|---|---|
| `CommandWatchdog` | `base_driver.py:46` | **独立线程**，非零速度指令超过 `cmd_watchdog_timeout`（0.5s）未被刷新 → 强制归零并 WARN。不经过 executor，故主线程阻塞/executor 卡死时仍有效；触发次数见 `BaseDriver.watchdog_trips` |
| `SensorHub.scan_age()` | `sensors.py:122` | 激光保鲜度（秒），无数据为 `inf`。**必须先查保鲜度再相信测距**：激光失效时 `sector_min_range` 返回 `None`，而开口判定与碰撞保护都把 `None` 当"通畅" |
| `MotionController.sensors_ok()` / `_stop_if_unhealthy()` | `motion_controller.py:177,188` | 激光年龄 > `scan_timeout`（0.5s）→ 禁止一切移动；`need_vision` 时相机年龄 > `vision_timeout`（2.0s）→ 禁止前进 |
| `advance()` 门禁 | `motion_controller.py:236` | **动作前 + 每个控制周期**各查一次；前进要求激光+相机，后退只要求激光 |
| `turn_to_heading()` / `strafe()` 门禁 | `motion_controller.py:325,367` | 动作前查激光，失效则不启动（避免把车停在半路） |
| `_run_explore()` 门禁 | `mission_manager.py:473` | 勘测路口前等待激光恢复 `sensor_wait_timeout`（3s）；仍失效 → FAULT。防止把"失效"写成拓扑里的假开口 |

**未覆盖**（仍需人工兜底）：进程被 `SIGKILL`/OOM/SIGHUP 时看门狗线程一并消失 → 只能物理
断电；转向/后退仍无激光守卫；无截止看门狗、无手柄接管。

验收：`pytest 91/91`（新增 `test_safety.py` 16 项）、`colcon build` 通过、
`maze_run --check-only` 正常、改动文件 `ament_flake8` 无违规。
