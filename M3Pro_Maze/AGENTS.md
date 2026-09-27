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

> ⚠️ `/cmd_vel` **无超时停车保护**，须 ≥10Hz 持续发布，停止时**显式发全 0 Twist**。

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

---

## 6. 编码约定

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

文档：`AGENTS.md`（本文件）、`PLAN.md`（实施方案）

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
