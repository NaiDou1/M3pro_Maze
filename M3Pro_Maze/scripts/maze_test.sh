#!/usr/bin/env bash
#
# M3Pro 迷宫自主探索系统 —— 一步启动测试脚本
#
# 位置：文档仓 `/home/jetson/Desktop/M3Pro_Maze/scripts/`，不参与 colcon 构建。
# 配套：`SAFETY.md`（安全审计与上机检查清单）、`ARCHITECTURE.md`（架构）、`AGENTS.md`（接口事实）。
#
# 设计原则：**默认不动车**。启动依赖链与自检是零风险的；只有显式 `--run` 才驱动底盘。
#
# 用法：
#   ./maze_test.sh                 # 默认：起依赖链 + 四项自检，不动车（推荐首次）
#   ./maze_test.sh --status        # 只看当前 ROS 图状态，什么都不启动
#   ./maze_test.sh --check         # 同上（默认档），显式写法
#   ./maze_test.sh --run           # 自检通过后，二次确认才真正开跑（会动车！）
#   ./maze_test.sh --run --yes     # 跳过打字确认（复跑用，仍需自检全绿）
#   ./maze_test.sh --calib hsv     # 起依赖后进标定：hsv | line_pose | motion | follow
#   ./maze_test.sh --no-deps ...   # 不管理依赖链（假定已手动起好）
#   ./maze_test.sh --stop          # 停止本脚本启动的后台依赖
#
# 已验证事实（2026-10-03 实测，来源 AGENTS.md / 本机 ros2 查询）：
#   * `/YB_Node` 是 STM32 固件节点（micro-ROS），自己订阅 `/arm6_joints`、`/cmd_vel`，
#     发布 `/odom_raw`、`/scan0`、`/scan1`、`/battery`、`/imu/data_raw`
#     → 机械臂与底盘无需额外驱动节点，只要 agent 在线即可
#   * `/scan0` + `/scan1` 由固件直出，必须经 `slam_mapping bringup` 合并滤波才有 `/scan`
#   * `M3Pro_demo camera_arm_kin.launch.py` **已包含相机启动**，不能再单独起相机（会撞车）
#   * 手柄自启链（`joy_ctrl` 等）会往 `/cmd_vel` 发指令抢控制权，跑任务前必须关掉
#
set -u -o pipefail

# ---------------------------------------------------------------- 常量
readonly SCRIPT_NAME="$(basename "$0")"
readonly WS_CAM="/home/jetson/yahboomcar_ws"
readonly WS_LIDAR="/home/jetson/M3Pro_ws"
readonly START_AGENT="/home/jetson/start_agent.sh"
readonly LOG_ROOT="${HOME}/maze_test_logs"
readonly STATE_FILE="${LOG_ROOT}/.deps.pid"
readonly STATE_DIR_FILE="${LOG_ROOT}/.deps.dir"
readonly ODOM_TOPIC="/odom_raw"
readonly SCAN_TOPIC="/scan"
readonly COLOR_TOPIC="/camera/color/image_raw"
readonly ARM_TOPIC="/arm6_joints"
readonly IK_SERVICE="/get_kinemarics"
readonly JOY_PATTERNS=("start_joy_controller.py" "yahboomcar_joy_launch.py" "yahboom_joy_M3Pro")

# ---------------------------------------------------------------- 输出
c_info()  { printf '\033[32m[信息]\033[0m %s\n' "$*"; }
c_warn()  { printf '\033[33m[警告]\033[0m %s\n' "$*"; }
c_err()   { printf '\033[31m[错误]\033[0m %s\n' "$*"; }
c_step()  { printf '\n\033[36m==> %s\033[0m\n' "$*"; }
die()     { c_err "$*"; exit 1; }

usage() {
    sed -n '3,25p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
}

# ---------------------------------------------------------------- 环境
setup_env() {
    export ROS_DOMAIN_ID=30
    [ -f /opt/ros/humble/setup.bash ] || die "未找到 /opt/ros/humble/setup.bash，ROS 2 Humble 未安装？"
    [ -f "${WS_CAM}/install/setup.bash" ] || die "未找到 ${WS_CAM}/install/setup.bash，请先 colcon build"
    [ -f "${WS_LIDAR}/install/setup.bash" ] || die "未找到 ${WS_LIDAR}/install/setup.bash，请先 colcon build"
    # ROS 的 setup.bash 不是 nounset-safe（会引用未定义变量），source 期间临时关掉 set -u
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    source "${WS_CAM}/install/setup.bash"
    # shellcheck disable=SC1091
    source "${WS_LIDAR}/install/setup.bash"
    set -u
    command -v ros2 >/dev/null 2>&1 || die "source 之后仍找不到 ros2 命令"
    c_info "环境就绪：ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，两个工作区已 source"
}

# 探测工具一：发布者数量（快，用于判活）。话题不存在或无发布者时输出 0。
# 实测：`ros2 topic info` 比 `echo --once` 快得多，且不受发现延迟影响。
topic_publishers() {
    local topic="$1" n
    n="$(timeout 8 ros2 topic info "${topic}" 2>/dev/null \
        | grep -i 'Publisher count' | grep -oE '[0-9]+' | head -1)"
    echo "${n:-0}"
}

# 探测工具一之补充：订阅者数量（命令类话题的判活依据）。
topic_subscribers() {
    local topic="$1" n
    n="$(timeout 8 ros2 topic info "${topic}" 2>/dev/null \
        | grep -i 'Subscription count' | grep -oE '[0-9]+' | head -1)"
    echo "${n:-0}"
}

# 探测工具二：实际收到一帧（慢但确凿）。实测 `echo --once` 需 1.7s 以上，
# 超时给 3s 会随机误判，故默认放宽到 8s。
probe_topic() {
    local topic="$1" timeout_s="${2:-8}"
    timeout "${timeout_s}" ros2 topic echo "${topic}" --once >/dev/null 2>&1
}

# 等话题真正出数据：先等发布者出现，再确认真收到帧
wait_topic() {
    local topic="$1" timeout_s="${2:-20}" t=0
    while [ "${t}" -lt "${timeout_s}" ]; do
        if [ "$(topic_publishers "${topic}")" -gt 0 ]; then
            if probe_topic "${topic}" 8; then
                c_info "话题就绪：${topic}（等待 ${t}s）"
                return 0
            fi
            c_warn "${topic} 有发布者但收不到数据（等待 ${t}s）"
        fi
        sleep 2; t=$((t + 2))
    done
    c_err "话题 ${topic} 在 ${timeout_s}s 内未就绪"
    return 1
}

wait_service() {
    local svc="$1" timeout_s="${2:-20}" t=0
    while [ "${t}" -lt "${timeout_s}" ]; do
        if ros2 service list 2>/dev/null | grep -qx "${svc}"; then
            c_info "服务就绪：${svc}（等待 ${t}s）"
            return 0
        fi
        sleep 1; t=$((t + 1))
    done
    c_err "服务 ${svc} 在 ${timeout_s}s 内未出现"
    return 1
}

# 机械臂关节话题必须已有订阅者（即 STM32 固件在线），否则关节指令发出去没人收
arm_subscriber_count() {
    ros2 topic info "${ARM_TOPIC}" 2>/dev/null \
        | grep -i "Subscription count" | grep -oE '[0-9]+' | head -1
}

wait_arm_subscriber() {
    local timeout_s="${1:-10}" t=0 count=0
    while [ "${t}" -lt "${timeout_s}" ]; do
        count="$(arm_subscriber_count)"; count="${count:-0}"
        if [ "${count}" -gt 0 ]; then
            c_info "机械臂关节话题已有 ${count} 个订阅者（STM32 固件在线）"
            return 0
        fi
        sleep 1; t=$((t + 1))
    done
    c_warn "${ARM_TOPIC} 无订阅者：机械臂不会动作，检查 micro-ROS agent 是否在线"
    return 1
}

# ---------------------------------------------------------------- 依赖链
dep_pids() { [ -f "${STATE_FILE}" ] && cat "${STATE_FILE}" || true; }

start_dep() {   # start_dep <名称> <日志名> <命令...>
    local name="$1" log="$2"; shift 2
    local dir; dir="$(cat "${STATE_DIR_FILE}" 2>/dev/null || echo "${LOG_ROOT}")"
    # 切到日志目录再启动：相机 SDK 会在 cwd 建 Log/ 目录，否则会污染调用者的工作目录
    pushd "${dir}" >/dev/null || true
    # setsid：让每个依赖链独占一个进程组（PGID == 记录的 PID）。
    # 停机时对**整个进程组**发 SIGINT，等价于终端里按 Ctrl-C，
    # 否则只杀 launch 父进程会让子节点变成孤儿继续跑（实测踩过）。
    setsid nohup "$@" >"${dir}/${log}.log" 2>&1 &
    local pid=$!
    popd >/dev/null || true
    echo "${pid}" >>"${STATE_FILE}"
    c_info "已启动 ${name}（pid=${pid}，独立进程组，日志 ${dir}/${log}.log）"
}

agent_alive() { pgrep -f "micro_ros_agent" >/dev/null 2>&1; }

ensure_agent() {
    if agent_alive; then
        c_info "micro-ROS agent 已在运行，跳过 start_agent.sh"
        return 0
    fi
    c_warn "未检测到 micro-ROS agent，尝试执行 ${START_AGENT}"
    [ -f "${START_AGENT}" ] || die "未找到 ${START_AGENT}"
    [ -n "${DISPLAY:-}" ] || die "无 DISPLAY，start_agent.sh 需要图形终端；请手动在桌面终端启动它"
    sh "${START_AGENT}" >/dev/null 2>&1 &
    sleep 8
    agent_alive || c_warn "agent 仍未出现，/odom_raw 可能收不到数据"
}

start_deps() {
    # 护栏一：上一次启动的依赖链还活着时**拒绝再起**。
    # 实测教训（2026-10-04）：脚本被连跑两次 → 两个相机容器抢同一台 USB 设备 →
    # libobsensor 抛 "Resource busy!" → 容器 abort（exit code 6）；且第二次
    # start_deps 会清空 .deps.pid，导致第一次的进程变成 --stop 看不见的孤儿。
    local old; old="$(dep_pids)"
    local alive=""
    local pid
    for pid in ${old}; do
        kill -0 "${pid}" 2>/dev/null && alive="${alive} ${pid}"
    done
    if [ -n "${alive}" ]; then
        c_err "上一次启动的依赖链仍在运行（pid:${alive}）"
        c_err "重复启动会让两个相机容器抢同一个 USB 设备，实测会崩溃（Resource busy）"
        die "请先执行：${SCRIPT_NAME} --stop"
    fi

    local stamp; stamp="$(date +%Y%m%d_%H%M%S)"
    local dir="${LOG_ROOT}/${stamp}"
    mkdir -p "${dir}"
    printf '%s' "${dir}" >"${STATE_DIR_FILE}"
    : >"${STATE_FILE}"
    c_info "本次日志目录：${dir}"

    c_step "1/3 下位机 micro-ROS（提供 /odom_raw、/scan0、/scan1、/battery）"
    ensure_agent
    wait_topic "${ODOM_TOPIC}" 25 || die "收不到 ${ODOM_TOPIC}，底盘链路不通，后续无意义"

    c_step "2/3 雷达链（合并 /scan0+/scan1 → /scan，并起 EKF）"
    start_dep "雷达链" radar ros2 launch slam_mapping bringup.launch.py
    wait_topic "${SCAN_TOPIC}" 40 || c_warn "${SCAN_TOPIC} 未就绪：安全门禁会拒绝一切移动"

    c_step "3/3 相机 + IK 服务（camera_arm_kin 已含相机，切勿再单独起相机）"
    start_dep "相机+IK" camera_ik ros2 launch M3Pro_demo camera_arm_kin.launch.py
    # 相机实测要 30~45s 才出流，且 USB2.0 下会自行断连重连（见下方日志检查）
    wait_topic "${COLOR_TOPIC}" 90 || c_warn "${COLOR_TOPIC} 未就绪：前进会被安全门禁拦下"
    wait_service "${IK_SERVICE}" 30 || c_warn "${IK_SERVICE} 未就绪：抓取不可用"
    wait_arm_subscriber 10 || true
    check_camera_log
}

# 实测（2026-10-04 日志）：Orbbec DaBai DCW2 挂在 USB2.0 上跑 color+depth+IR 三路流，
# 会在启动约 27s 后 onDeviceDisconnected、约 7s 后自行重连。断连窗口内自检会误报
# “未收到相机图像”。这里只报事实与建议，不擅自改厂商 launch。
check_camera_log() {
    local dir; dir="$(cat "${STATE_DIR_FILE}" 2>/dev/null || echo "${LOG_ROOT}")"
    local log="${dir}/camera_ik.log"
    [ -f "${log}" ] || return 0
    if grep -q "onDeviceDisconnected" "${log}" 2>/dev/null; then
        c_warn "相机日志出现 onDeviceDisconnected（USB 掉线重连）——自检可能因此偶发误报相机 FAIL"
        c_warn "现场对策：等 10s 重试自检；换 USB3.0 口/直连不 Hub；或与组委会确认相机配置"
        c_warn "详见：${log}"
    fi
}

stop_deps() {
    local pids; pids="$(dep_pids)"
    if [ -z "${pids}" ]; then
        c_info "没有本脚本启动的依赖进程（${STATE_FILE} 为空）"
        return 0
    fi
    c_step "停止本脚本启动的依赖链（对进程组发 SIGINT → 必要时 SIGTERM，绝不 -9）"

    local pid pgid
    # 第一轮：SIGINT 整个进程组（等价于终端 Ctrl-C，launch 会优雅关闭子节点）
    for pid in ${pids}; do
        kill -0 "${pid}" 2>/dev/null || continue
        pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
        [ -n "${pgid}" ] || pgid="${pid}"
        c_info "SIGINT → 进程组 ${pgid}（含 launch 与其全部子节点）"
        kill -INT "-${pgid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    done

    # 给足优雅退出时间：相机容器与 EKF 关闭需要数秒
    local waited=0 alive=0
    while [ "${waited}" -lt 15 ]; do
        alive=0
        for pid in ${pids}; do
            kill -0 "${pid}" 2>/dev/null && alive=1
        done
        [ "${alive}" -eq 0 ] && break
        sleep 1; waited=$((waited + 1))
    done

    # 第二轮：仍未退出则 SIGTERM 整个进程组
    if [ "${alive}" -ne 0 ]; then
        c_warn "等待 ${waited}s 仍有进程存活，改用 SIGTERM"
        for pid in ${pids}; do
            kill -0 "${pid}" 2>/dev/null || continue
            pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
            [ -n "${pgid}" ] || pgid="${pid}"
            kill -TERM "-${pgid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
        done
        sleep 3
    fi

    : >"${STATE_FILE}"
    c_info "依赖链停止流程结束（耗时 ${waited}s）"

    # 残留自检：孤儿节点是"停不干净"的典型症状，必须报出来。
    # 实测要点：相机以 `component_container`（rclcpp_components）形式运行，
    # 进程名里没有 "orbbec"/"camera"，漏掉它就会留下一个隐形的相机发布者。
    local left
    # 排除本脚本自身与父进程，避免命令行里恰好含关键字时自匹配（实测踩过）
    left="$(pgrep -af '[l]aserscan_multi_merger|[l]aser_filter_node|[e]kf_filter_node|[i]mu_filter_madgwick|[k]in_srv|[c]omponent_container|[r]obot_state_publisher' 2>/dev/null \
        | awk -v me="$$" -v pp="${PPID:-0}" '$1 != me && $1 != pp' | cut -c1-80)"
    if [ -n "${left}" ]; then
        c_warn "仍有依赖节点存活（可能是本次或上次遗留）："
        printf '%s\n' "${left}" | sed 's/^/      /'
        c_warn "处理：对上面每个 PID 发一次 SIGINT；确认无用后再考虑 SIGTERM，不要 -9"
    else
        c_info "无依赖节点残留"
    fi
}

# ---------------------------------------------------------------- 状态
show_status() {
    c_step "当前 ROS 图状态（ROS_DOMAIN_ID=${ROS_DOMAIN_ID}）"
    # 话题列表只取一次，避免逐个询问拖慢状态显示
    local topic_list; topic_list="$(timeout 10 ros2 topic list 2>/dev/null)"
    printf '  节点：\n'; timeout 10 ros2 node list 2>/dev/null | sed 's/^/    /' | head -12
    # 数据类话题：判据是"能收到帧"
    printf '  数据类话题（看是否真出数据）：\n'
    local topic pubs
    for topic in "${ODOM_TOPIC}" "${SCAN_TOPIC}" "${COLOR_TOPIC}" /battery; do
        if ! printf '%s\n' "${topic_list}" | grep -qx "${topic}"; then
            printf '    [不存在] %s\n' "${topic}"
            continue
        fi
        pubs="$(topic_publishers "${topic}")"
        if [ "${pubs}" -eq 0 ]; then
            printf '    [无发布者] %s\n' "${topic}"
        elif probe_topic "${topic}" 8; then
            printf '    [有数据] %s\n' "${topic}"
        else
            printf '    [有发布者但收不到数据] %s\n' "${topic}"
        fi
    done

    # 命令类话题：判据是"谁在听/有几个在发"，收不到数据是正常的
    printf '  命令类话题（看订阅者在不在、有没有人抢）：\n'
    for topic in /cmd_vel "${ARM_TOPIC}"; do
        printf '    %-16s 发布者 %s，订阅者 %s\n' "${topic}" \
            "$(topic_publishers "${topic}")" "$(topic_subscribers "${topic}")"
    done
    printf '      （/cmd_vel 发布者 >1 = 有别的节点在抢控制权；%s 订阅者 0 = 固件不在线）\n' \
        "${ARM_TOPIC}"
    printf '  IK 服务：'
    if ros2 service list 2>/dev/null | grep -qx "${IK_SERVICE}"; then
        printf '[在线] %s\n' "${IK_SERVICE}"
    else
        printf '[离线] %s（抓取不可用）\n' "${IK_SERVICE}"
    fi
    local batt
    batt="$(timeout 8 ros2 topic echo /battery --once 2>/dev/null | grep -m1 data | awk '{print $2}')"
    if [ -n "${batt}" ]; then
        printf '  电池读数（/battery，Float32，量纲未核验，仅供参考）：%s\n' "${batt}"
    fi
    printf '  手柄自启链：'
    if pgrep -f "${JOY_PATTERNS[0]}" >/dev/null 2>&1 || pgrep -f "${JOY_PATTERNS[2]}" >/dev/null 2>&1; then
        printf '\033[33m正在运行（跑任务前会自动关闭）\033[0m\n'
    else
        printf '未运行\n'
    fi
}

# ---------------------------------------------------------------- 安全
kill_joy_chain() {
    local p
    for p in "${JOY_PATTERNS[@]}"; do
        if pgrep -f "${p}" >/dev/null 2>&1; then
            c_info "关闭手柄自启进程：${p}"
            pkill -f "${p}" || true
        fi
    done
    sleep 1
}

self_check() {
    c_step "四项硬件自检（maze_run --check-only，不动车）"
    if ros2 run maze_explorer maze_run --check-only; then
        c_info "自检全部通过"
        return 0
    fi
    c_err "自检未通过，详见上面的逐项提示"
    return 1
}

# 自检带一次重试：相机在 USB2.0 下会掉线重连（实测 27s 断、7s 恢复），
# 恰好撞上自检就会误报相机 FAIL。这种情况重试一次即可，不必让用户重跑整个流程。
self_check_with_retry() {
    self_check && return 0
    c_warn "首次自检未过：等 10s 让相机/雷达稳定后重试一次"
    c_warn "（若日志出现 onDeviceDisconnected，属 USB 掉线重连，非配置错误）"
    sleep 10
    self_check && return 0
    return 1
}

print_stop_guide() {
    cat <<'EOF'

--------------------------------------------------------------------
如何停止（务必照做，详见 SAFETY.md）
  1) 在本终端按 **一次** Ctrl-C，等它自己退出（几秒），
     日志应出现 "任务完成：..." 或 "任务中断：..."；
  2) 另开终端提前跑 `ros2 topic echo /cmd_vel`，
     停机后最后一条必须是全 0；只要不是全 0，**立刻断电**；
  3) 绝对不要 `kill -9`、不要直接关终端窗口（SIGHUP）——
     这两种情况归零指令不会发出，车会保持最后速度继续跑；
  4) 停不下来的应急动作：先把车提起来（轮子离地），再断电；
  5) 停依赖链：`./maze_test.sh --stop`。
--------------------------------------------------------------------
EOF
}

# ---------------------------------------------------------------- 主流程
MODE="check"
CALIB_MODE=""
ASSUME_YES=0
NO_DEPS=0

while [ $# -gt 0 ]; do
    case "$1" in
        --check)      MODE="check" ;;
        --run)        MODE="run" ;;
        --status)     MODE="status" ;;
        --stop)       MODE="stop" ;;
        --calib)      MODE="calib"; CALIB_MODE="${2:-}"; shift ;;
        --no-deps)    NO_DEPS=1 ;;
        --yes|-y)     ASSUME_YES=1 ;;
        -h|--help)    usage ;;
        *)            die "未知参数：$1（用 --help 查看用法）" ;;
    esac
    shift
done

mkdir -p "${LOG_ROOT}"

# 参数校验放在最前：参数不对就别白花一分钟起依赖链
if [ "${MODE}" = "calib" ]; then
    case "${CALIB_MODE}" in
        hsv|line_pose|motion|follow) ;;
        *) die "--calib 需要参数：hsv | line_pose | motion | follow" ;;
    esac
fi

setup_env

# 全程落盘：--run/--calib 的会话与任务输出必须留档，否则出问题只能靠回忆终端
if [ "${MODE}" = "run" ] || [ "${MODE}" = "calib" ]; then
    SESSION_LOG="${LOG_ROOT}/session_$(date +%Y%m%d_%H%M%S)_${MODE}.log"
    # shellcheck disable=SC2094
    exec > >(tee -a "${SESSION_LOG}") 2>&1
    c_info "本次会话完整日志：${SESSION_LOG}"
fi

case "${MODE}" in
    status)
        show_status
        exit 0
        ;;
    stop)
        stop_deps
        exit 0
        ;;
esac

if [ "${NO_DEPS}" -eq 0 ]; then
    start_deps
else
    c_info "--no-deps：不启动依赖链，假定已手动起好"
fi

c_step "依赖链状态总览"
show_status

if ! wait_topic "${ODOM_TOPIC}" 5; then
    die "${ODOM_TOPIC} 无数据，先解决底盘链路"
fi

self_check_with_retry || die "自检未通过，已中止（没有下发任何运动指令）"

case "${MODE}" in
    check)
        c_info "自检档完成：已启动依赖链并全部就绪，**未下达任何运动指令**"
        c_info "要真跑：${SCRIPT_NAME} --run；只看状态：${SCRIPT_NAME} --status"
        print_stop_guide
        ;;
    calib)
        case "${CALIB_MODE}" in
            hsv|line_pose)
                if [ -z "${DISPLAY:-}" ]; then
                    c_err "标定 ${CALIB_MODE} 需要图形界面（cv2.imshow），当前无 DISPLAY（SSH 会话）"
                    c_info "三个可选办法："
                    c_info "  1) 在 Jetson 本机屏幕前的桌面终端里跑本脚本"
                    c_info "  2) 用 ssh -X / -Y 登录（本机 X server 存在：/tmp/.X11-unix/X0）"
                    c_info "  3) 不需要界面：--calib follow（纯巡线）与 --calib motion（转角）"
                    die "已中止（未下达任何运动指令）"
                fi
                ;;
        esac
        if [ "${CALIB_MODE}" = "motion" ]; then
            c_warn "motion 标定按固定 3s/8s 开环驱动且无碰撞保护 —— 确认车在≥1m 空地"
        fi
        kill_joy_chain
        c_step "进入标定：${CALIB_MODE}"
        ros2 run maze_explorer maze_calib "${CALIB_MODE}"
        print_stop_guide
        ;;
    run)
        # 护栏二：已经有任务在跑时拒绝再起（两个 mission_manager 会互抢 /cmd_vel）
        if pgrep -f '[m]aze_explorer/mission_manager|[r]os2 run maze_explorer maze_run' >/dev/null 2>&1; then
            die "检测到已有任务在运行，拒绝重复启动（先 Ctrl-C 停掉它，或用 --status 查看）"
        fi

        # 运动前的最后一道闸：说清楚风险并要一次显式确认
        cat <<'EOF'

====================================================================
即将真跑，请先逐条确认：
  [ ] 车已放在场地/空地，轮子着地，周围无人手在机械臂活动范围内
  [ ] 手放在电源开关上（软件看门狗管不住 kill -9，只有断电是终极手段）
  [ ] 黑线/四色 HSV、line_pose、转角系数已按现场标定过（见 AGENTS §10.4）
====================================================================
EOF
        if [ "${ASSUME_YES}" -eq 0 ]; then
            printf '确认无误请输入 RUN 并回车（大小写不限，r/y 也可；其他任何输入都中止）：'
            read -r answer
            # 大小写与简写都接受：实测有人输入小写 run 被旧版拒绝，误以为“跑了但没动”
            case "$(printf '%s' "${answer}" | tr '[:upper:]' '[:lower:]')" in
                run|r|yes|y) c_info "已确认：${answer}" ;;
                *) die "未确认（收到 '${answer}'，需要 RUN），已中止（没有下达任何运动指令）" ;;
            esac
        fi
        kill_joy_chain
        c_step "启动任务（Ctrl-C 停止；本脚本会在任务退出后打印停机核对清单）"
        [[ -n "${SESSION_LOG:-}" ]] && c_info "任务输出同时写入：${SESSION_LOG}"
        # MISSION_CMD 覆盖点仅用于**脚本自测**（例如设为 true 可在不驱动底盘的前提下
        # 验证确认/日志/收尾路径），日常不要改它。
        mission_cmd="${MAZE_TEST_MISSION_CMD:-ros2 run maze_explorer maze_run}"
        # 捕获 INT 只为在子进程退出后继续跑收尾逻辑，不吞掉子进程的信号
        trap 'c_warn "收到 Ctrl-C，等待任务退出…"' INT
        # shellcheck disable=SC2086
        ${mission_cmd}
        rc=$?
        trap - INT
        c_step "任务已退出（返回码 ${rc}）"
        if [ "${rc}" -ne 0 ]; then
            c_warn '非 0 退出：若日志里没有「任务完成/任务中断」，说明是异常退出，请核对 /cmd_vel 最后一条，必要时断电'
            c_warn "排查入口：${SESSION_LOG:-（本次无会话日志）}；ROS 节点日志在 ~/.ros/log/"
        fi
        print_stop_guide
        ;;
esac
