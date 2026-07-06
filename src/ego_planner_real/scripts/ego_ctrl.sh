#!/usr/bin/env bash
#=============================================================================
#  ego_ctrl.sh — EGO-Planner 实体飞行控制/监控脚本
#
#  用法:
#    ./ego_ctrl.sh goal 5 3 1.5        # 发送全局目标 (x, y, z)
#    ./ego_ctrl.sh body 3 0 0          # 机体坐标目标 (前 3m)
#    ./ego_ctrl.sh hover               # 原地悬停
#    ./ego_ctrl.sh land                # 降落
#    ./ego_ctrl.sh mon pos             # 监控位置设定点
#    ./ego_ctrl.sh mon vel             # 监控速度设定点
#    ./ego_ctrl.sh mon reached         # 等待到达目标
#    ./ego_ctrl.sh mon all             # 监控全部输出
#    ./ego_ctrl.sh mon odom            # 监控里程计
#    ./ego_ctrl.sh rviz_goal           # 进入 RViz 点击目标模式
#    ./ego_ctrl.sh wp 5 3 1.5 10 5 2.0 15 3 2.5  # 航点序列
#    ./ego_ctrl.sh list                # 列出所有活跃话题
#    ./ego_ctrl.sh info                # 显示接口文档摘要
#=============================================================================

set -e

# ---- 话题前缀 (根据 drone_id 调整) ----
DRONE_ID="${DRONE_ID:-0}"
NODE="drone_${DRONE_ID}_ego_planner_node"
TRAJ="drone_${DRONE_ID}_traj_server"
GOAL_TOPIC="/ego_planner/goal"
GOAL_BODY_TOPIC="/${NODE}/goal_body"
GOAL_REACHED_TOPIC="/${NODE}/goal_reached"
CURRENT_POSE_TOPIC="/${NODE}/current_pose"
REPLAN_START_TOPIC="/${NODE}/replan_start"
POS_SP_TOPIC="/${TRAJ}/position_setpoint"
VEL_SP_TOPIC="/${TRAJ}/velocity_setpoint"
YAW_SP_TOPIC="/${TRAJ}/yaw_setpoint"
POS_CMD_TOPIC="/position_cmd"

# ---- 颜色 ----
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

die() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
info(){ echo -e "${GREEN}[INFO]${NC} $*"; }
warn(){ echo -e "${YELLOW}[WARN]${NC} $*"; }

usage() {
    cat << EOF
${CYAN}ego_ctrl.sh${NC} — EGO-Planner 实体飞行控制脚本

${YELLOW}输入命令:${NC}
  goal  <x> <y> <z>    全局目标点 (odom 坐标系)
  body  <x> <y> <z>    机体坐标系目标 (前x 左y 上z)
  hover                 原地悬停 (当前位置)
  land                  降落 (z=0.2)
  rviz_goal             进入 RViz 点击目标交互模式
  wp    <x1 y1 z1> ... 航点序列 (至少 3 个坐标)

${YELLOW}监控命令:${NC}
  mon pos               监控位置设定点 (PoseStamped)
  mon vel               监控速度设定点 (TwistStamped)
  mon yaw               监控偏航设定点 (Float32)
  mon cmd               监控完整指令 (PositionCommand)
  mon odom              监控里程计 (current_pose)
  mon reached           等待目标到达通知
  mon all               同时监控位置+速度+偏航+到达状态
  mon replan            监控重规划起点 (诊断用)

${YELLOW}工具命令:${NC}
  list                  列出所有 ego-planner 相关话题
  info                  显示接口文档摘要
  nodes                 列出运行中的 ego-planner 节点

${YELLOW}环境变量:${NC}
  DRONE_ID=0            无人机 ID (默认 0)

${YELLOW}示例:${NC}
  ./ego_ctrl.sh goal 5 3 1.5
  ./ego_ctrl.sh body 3 0 0
  ./ego_ctrl.sh mon all
  DRONE_ID=1 ./ego_ctrl.sh goal 10 5 2.0
EOF
}

# =========================================================================
#  输入命令
# =========================================================================

cmd_goal() {
    local x="$1" y="$2" z="${3:-1.0}"
    [[ -z "$x" || -z "$y" ]] && die "用法: goal <x> <y> [z]"
    info "发送全局目标: (${x}, ${y}, ${z})"
    ros2 topic pub --once "$GOAL_TOPIC" geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'odom'}, pose: {position: {x: $x, y: $y, z: $z}}}" \
        2>/dev/null && info "目标已发送" || die "发送失败, 检查规划器是否运行"
}

cmd_body() {
    local x="$1" y="$2" z="${3:-0.0}"
    [[ -z "$x" || -z "$y" ]] && die "用法: body <前x> <左y> [上z]"
    info "发送机体目标: 前${x}m 左${y}m 上${z}m"
    ros2 topic pub --once "$GOAL_BODY_TOPIC" geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'body'}, pose: {position: {x: $x, y: $y, z: $z}}}" \
        2>/dev/null && info "目标已发送" || die "发送失败"
}

cmd_hover() {
    info "原地悬停 (发送当前位置)"
    ros2 topic pub --once "$GOAL_BODY_TOPIC" geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'body'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}" \
        2>/dev/null && info "悬停指令已发送" || die "发送失败"
}

cmd_land() {
    info "降落 (z=0.2m)"
    ros2 topic pub --once "$GOAL_BODY_TOPIC" geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'body'}, pose: {position: {x: 0.0, y: 0.0, z: -0.3}}}" \
        2>/dev/null && info "降落指令已发送" || die "发送失败"
}

cmd_wp() {
    [[ $# -lt 3 ]] && die "用法: wp <x1 y1 z1> [x2 y2 z2] ..."
    local count=0
    while [[ $# -ge 3 ]]; do
        local x="$1" y="$2" z="$3"; shift 3
        info "航点 $((++count)): (${x}, ${y}, ${z})"
        ros2 topic pub --once "$GOAL_TOPIC" geometry_msgs/msg/PoseStamped \
            "{header: {frame_id: 'odom'}, pose: {position: {x: $x, y: $y, z: $z}}}"
        if [[ $# -ge 3 ]]; then
            info "等待到达..."
            ros2 topic echo --once "$GOAL_REACHED_TOPIC" 2>/dev/null | grep -q 'data: true' \
                && info "已到达, 发送下一航点" \
                || warn "超时, 继续发送下一航点"
            sleep 0.5
        fi
    done
    info "航点序列完成"
}

cmd_rviz_goal() {
    info "进入 RViz 点击目标模式"
    echo "  1. 在 RViz 工具栏选择 '2D Goal Pose'"
    echo "  2. 在地图上点击目标位置"
    echo "  3. 规划器自动规划并执行"
    echo ""
    echo "  同时监控到达状态..."
    ros2 topic echo "$GOAL_REACHED_TOPIC" 2>/dev/null | while read line; do
        if echo "$line" | grep -q 'data: true'; then
            info "到达目标!"
        elif echo "$line" | grep -q 'data: false'; then
            info "正在飞行..."
        fi
    done
}

# =========================================================================
#  监控命令
# =========================================================================

cmd_mon() {
    case "${1:-all}" in
        pos)
            info "监控位置设定点"
            ros2 topic echo "$POS_SP_TOPIC"
            ;;
        vel)
            info "监控速度设定点"
            ros2 topic echo "$VEL_SP_TOPIC"
            ;;
        yaw)
            info "监控偏航设定点"
            ros2 topic echo "$YAW_SP_TOPIC"
            ;;
        cmd)
            info "监控完整指令"
            ros2 topic echo "$POS_CMD_TOPIC"
            ;;
        odom)
            info "监控里程计 (规划器收到的实时位置)"
            ros2 topic echo "$CURRENT_POSE_TOPIC"
            ;;
        reached)
            info "等待目标到达通知 (Ctrl+C 退出)"
            ros2 topic echo "$GOAL_REACHED_TOPIC" 2>/dev/null | while read line; do
                if echo "$line" | grep -q 'data: true'; then
                    info "✅ 已到达目标!"
                fi
            done
            ;;
        replan)
            info "监控重规划起点"
            ros2 topic echo "$REPLAN_START_TOPIC"
            ;;
        all)
            info "同时监控: 位置设定点 + 速度设定点 + 偏航 + 到达状态"
            info "=============================================="
            gnome-terminal --tab -t "位置设定点" -- bash -c "ros2 topic echo $POS_SP_TOPIC; exec bash" 2>/dev/null || \
            xterm -title "位置设定点" -e "ros2 topic echo $POS_SP_TOPIC; bash" 2>/dev/null || \
            ros2 topic echo "$POS_SP_TOPIC"
            ;;
        *)
            die "未知监控项: $1 (可用: pos vel yaw cmd odom reached replan all)"
            ;;
    esac
}

# =========================================================================
#  工具命令
# =========================================================================

cmd_list() {
    info "EGO-Planner 相关话题:"
    echo ""
    echo -e "${YELLOW}── 输入 ──${NC}"
    ros2 topic list 2>/dev/null | grep -E "ego_planner.*goal|move_base_simple" || echo "  (无)"
    echo ""
    echo -e "${YELLOW}── 输出 (控制指令) ──${NC}"
    ros2 topic list 2>/dev/null | grep -E "traj_server.*(position_setpoint|velocity_setpoint|yaw_setpoint)" || echo "  (无)"
    echo ""
    echo -e "${YELLOW}── 输出 (状态) ──${NC}"
    ros2 topic list 2>/dev/null | grep -E "goal_reached|current_pose|replan_start" || echo "  (无)"
    echo ""
    echo -e "${YELLOW}── 可视化 ──${NC}"
    ros2 topic list 2>/dev/null | grep -E "plan_vis|occupancy_inflate" || echo "  (无)"
    echo ""
    echo -e "${YELLOW}── 全部 ──${NC}"
    ros2 topic list 2>/dev/null | grep -E "ego_planner|traj_server.*drone" || echo "  (无)"
}

cmd_nodes() {
    info "运行中的节点:"
    ros2 node list 2>/dev/null | grep -E "ego_planner|traj_server|drone_model|robot_state|odin" || echo "  (无)"
}

cmd_info() {
    cat << EOF
${CYAN}═══════════════════════════════════════════${NC}
${CYAN}  EGO-Planner 实体飞行 — 接口速查${NC}
${CYAN}═══════════════════════════════════════════${NC}

${GREEN}输入接口:${NC}
  /ego_planner/goal                 全局目标 (odom 坐标系)
  /drone_0_ego_planner_node/goal_body  机体目标 (相对位置)
  /move_base_simple/goal            RViz 2D Nav Goal

${GREEN}输出接口 (traj_server, 100Hz):${NC}
  /drone_0_traj_server/position_setpoint  位置 + 偏航 (PoseStamped)
  /drone_0_traj_server/velocity_setpoint  线速度 + 角速率 (TwistStamped)
  /drone_0_traj_server/yaw_setpoint       偏航角 (Float32)
  /position_cmd                           完整指令 (PositionCommand)

${GREEN}状态反馈:${NC}
  /drone_0_ego_planner_node/goal_reached   到达通知 (Bool)
  /drone_0_ego_planner_node/current_pose   当前位姿 (PoseStamped)

${GREEN}常用命令:${NC}
  ./ego_ctrl.sh goal 5 3 1.5      飞向 (5, 3, 1.5)
  ./ego_ctrl.sh body 3 0 0        向前飞 3m
  ./ego_ctrl.sh hover             悬停
  ./ego_ctrl.sh mon all           监控全部输出
  ./ego_ctrl.sh list              列出所有话题
EOF
}

# =========================================================================
#  主入口
# =========================================================================

case "${1:-}" in
    goal)      shift; cmd_goal "$@" ;;
    body)      shift; cmd_body "$@" ;;
    hover)     cmd_hover ;;
    land)      cmd_land ;;
    wp)        shift; cmd_wp "$@" ;;
    rviz_goal) cmd_rviz_goal ;;
    mon)       shift; cmd_mon "$@" ;;
    list)      cmd_list ;;
    info)      cmd_info ;;
    nodes)     cmd_nodes ;;
    -h|--help|help|"") usage ;;
    *)         die "未知命令: $1 (运行 ./ego_ctrl.sh 查看帮助)" ;;
esac
