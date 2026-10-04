# M3Pro 迷宫自主探索与方块收集系统

给 ROSMASTER M3 Pro 小车做的**二维迷宫自主探索 + 彩色方块收集**系统：小车从入口进入 7×7 迷宫，自主建图、循迹、识别红/绿/黄/蓝四种方块（各 2 个，共 8 个），用机械臂逐块抓入车载收集筐，最后带全部方块从出口驶出。

成绩 = **实际用时 + 漏取罚时**（罚时按方块深度与颜色计算），所以目标是在保证 8 块全部到达出口的前提下把时间压到最短。

---

## 目录

- [比赛任务摘要](#比赛任务摘要)
- [方案要点](#方案要点)
- [文档导航](#文档导航)
- [仓库结构](#仓库结构)
- [快速开始](#快速开始)
- [开发流程](#开发流程)
- [当前状态](#当前状态)

---

## 比赛任务摘要

| 项 | 值 |
|---|---|
| 场地 | 2.8m × 2.8m，7 格 × 7 格，单通道宽 40cm，挡板高 55cm |
| 车体 | 21.5cm(宽) × 27.5cm(长)，通道内单侧余量仅 9.25cm |
| 迷宫 | 树形结构，一个入口一个出口，通道正中贴黑色引导线 |
| 目标 | 红/绿/黄/蓝方块各 2 个，分布在不同深度；由本队自备，只核验颜色与数量 |
| 计分 | 实际完成时间 + 漏取罚时（深度 × 颜色 × 各队基准速度，系数赛前未知） |
| 硬约束 | 任何状态横向宽度 ≤ 21.5cm；总高 ≤ 55cm；不得损毁迷宫 |

---

## 方案要点

1. **不做栅格 SLAM**：场地是规则 7×7 网格，用激光测墙直接构建"格—连通性"拓扑图，比连续建图更抗里程计漂移。
2. **离散运动原语**：走一格 / 转 90° / 掉头，每格重新对齐，误差不累积。
3. **激光居中为主、视觉循迹为辅**：40cm 通道两侧挡板对称居中，不受黑线被方块遮挡的影响；相机保留作方块识别与路口识别。
4. **相机装在机械臂上**（本硬件的硬约束）：必须在固定的"巡线姿态"下完成方块三维定位，停稳后机械臂才动作。
5. **遇块即抓 + 先深后浅**：抓取节拍压缩到 6~8s/块（`arm6_joints.time` 计时契约 + 视觉复查，本机实测无舵机反馈）。
6. **降级返航**：任何不可恢复故障或截止看门狗触发，立刻携带已收方块走最短路去出口，绝不原地等待——到不了出口，收益归零。

完整决策与实施阶段见 [`PLAN.md`](PLAN.md)。

---

## 文档导航

| 文件 | 给谁看 | 内容 |
|---|---|---|
| `README.md` | 人 | 本文件：项目是什么、怎么跑、怎么参与开发 |
| `AGENTS.md` | AI agent / 开发者 | 硬件接口事实速查、文风与代码/提交规范、陷阱清单、待验证清单 |
| `PLAN.md` | AI agent / 开发者 | 技术决策、架构设计、数据结构、分阶段实施与验收标准 |

> `AGENTS.md` 中的硬件事实均为本机验证结果；未验证的条目一律放在文末「待验证清单」，不当作事实使用。

---

## 仓库结构

双仓库、各自独立，均**只做本地提交**（不配 remote、不 push）：

| 仓库 | 路径 | 内容 |
|---|---|---|
| 文档仓库 | `/home/jetson/Desktop` | 本目录的 `README.md` / `AGENTS.md` / `PLAN.md` |
| 代码仓库 | `/home/jetson/yahboomcar_ws/src/maze_explorer` | 自研 ROS 2 包源码（阶段 1 时初始化） |

代码目录规划：

```
maze_explorer/
├── package.xml / setup.py
├── config/          maze_params.yaml   场地/速度/激光扇区
│                    hsv_params.yaml    黑线 + 四色阈值（现场标定）
│                    arm_poses.yaml     巡线/抓取/放置位姿、外参
├── launch/          maze_bringup.launch.py   一键启动
├── maze_explorer/   base_driver   底盘：20Hz 发布 cmd_vel、订阅 odom_raw
│                    sensors       RGB-D 时间同步、激光扇区取距
│                    line_detector 黑线偏差 + 方块压线遮挡
│                    block_detector 四色方块三维定位（滑行预检 + 停车精定位）
│                    grid_mapper   7×7 网格拓扑、出口判据
│                    dfs_planner   DFS 探索、以出口为终点的遍历
│                    motion_controller 循迹/走格/转向/掉头
│                    arm_controller    关节下发与 IK 调用
│                    grasp_fsm     抓取状态机（重试 1 次）
│                    mission_manager   顶层编排 + 降级返航
│                    score_logger  用时与罚时打点报表
│                    calibration_tool  HSV/外参/转角标定工具
└── scripts/         run_maze.sh   环境准备 + 启动 + rosbag 录制
```

---

## 快速开始

**环境**（已验证）：NVIDIA Jetson Orin，Ubuntu 22.04.5，ROS 2 Humble，`ROS_DOMAIN_ID=30`。

### 一键（推荐首次）

```bash
M3Pro_Maze/scripts/maze_test.sh            # 起依赖链 + 四项自检，不动车（约 1~2 分钟）
M3Pro_Maze/scripts/maze_test.sh --run      # 自检全绿后，打字确认 RUN 才真跑
M3Pro_Maze/scripts/maze_test.sh --stop     # 停掉脚本起的依赖链
M3Pro_Maze/scripts/maze_test.sh --status   # 只看当前 ROS 图，不启动任何东西
```

脚本已处理三件事：`ROS_DOMAIN_ID=30` 与两个工作区 source、**相机只起一次**
（`camera_arm_kin.launch.py` 本身已含相机，单独再起会撞车）、依赖链停机时对整个
进程组发 SIGINT（只杀 launch 父进程会留下孤儿节点）。

### 手动（等价步骤）

```bash
# 1. 环境（必须按序 source 两个工作区）
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ws/install/setup.bash
source /home/jetson/M3Pro_ws/install/setup.bash

# 2. 启动链（严格按序）
sh /home/jetson/start_agent.sh                          # 下位机 micro-ROS（提供 /odom_raw、/scan0/1、/battery）
ros2 launch slam_mapping bringup.launch.py              # 雷达链 + IMU 滤波 + EKF → /scan
ros2 launch M3Pro_demo camera_arm_kin.launch.py         # 相机 + IK 服务（勿再单独起相机）
ros2 run maze_explorer maze_run --check-only            # 四项自检，不动车
ros2 run maze_explorer maze_run                         # 真跑（手放电源开关上）

# 3. 构建（只构建自研包）
colcon build --packages-select maze_explorer
```

运行前三个坑（详见 `AGENTS.md` §5 与 `SAFETY.md`）：

- 关掉开机自启的手柄节点 `yahboom_joy_M3Pro`，否则 `joy_ctrl` 会与本体抢 `/cmd_vel`；
- 未设 `ROS_DOMAIN_ID=30` 或漏 source 任一工作区时，看不到任何话题；
- 相机在 USB2.0 下会自行掉线重连（启动约 27s 后），窗口内自检会**误报**相机 FAIL，等 10s 重试。

---

## 开发流程

每个可验证步骤走同一个循环：

```
写码（AGENTS §6 风格与节点约定）
→ colcon build --packages-select maze_explorer
→ pytest（纯逻辑离线单测，mock 传感器）+ flake8/pep257 过检
→ 实机小步验证（rosbag 录制，按 PLAN §八 验收标准打勾）
→ 文档回写（验证过的事实进正文，没过的进待验证清单）
→ git 小步提交
```

提交信息格式：`<type>(<scope>): <中文简述>`，如 `feat(line_detector): 黑线中心提取与压线遮挡状态`。
完整 type 表与示例见 `AGENTS.md` §9。

实施共 8 个阶段（含刻意前置的"最小端到端切片"，用于提前暴露 IK/抓取包络/收集筐位姿风险），见 `PLAN.md` §八。

---

## 当前状态

- **文档阶段**（2026-09-27）：`AGENTS.md` / `PLAN.md` 已定稿，硬件接口事实已核对。
- **代码未开工**：`/home/jetson/yahboomcar_ws/src/maze_explorer` 尚未创建，下一步执行 `PLAN.md` §八 阶段 1。
