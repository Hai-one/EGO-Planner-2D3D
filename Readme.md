# EGO-Planner for Odin

基于 [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 修改，适配 **Odin1 飞控 + Livox Mid-360 LiDAR + RK3588** 的实体无人机导航系统。

Modified from [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm), adapted for **Odin1 FC + Livox Mid-360 + RK3588** real-world drone navigation.

---

## 硬件平台 / Hardware

| 组件 | 型号 |
|------|------|
| 机载计算机 | RK3588 (LubanCat) |
| 飞控 | Odin1 |
| LiDAR | Livox Mid-360 |
| 系统 | Ubuntu 22.04 |
| 中间件 | ROS 2 Humble |

---

## 新增功能 / New Features

相比上游仓库，本仓库新增/修复：

- ✅ **ROS 2 Action 导航接口** — `NavigateToPose.action`，支持反馈、取消、错误码
- ✅ **Z 轴规划使能开关** — `fsm/enable_z_planning`，可锁定高度仅做 XY 平面导航
- ✅ **独立 Z 轴膨胀半径** — `grid_map/obstacles_inflation_z`，XY/Z 膨胀分开控制
- ✅ **Mid-360 LiDAR 点云直连** — 无需深度相机，点云管线独立工作
- ✅ **可视化截断高度可配** — `grid_map/visualization_truncate_height` 放入 launch 参数
- ✅ **时间源同步修复** — 所有节点统一使用 `node->get_clock()`，修复 `[1 != 2]` 崩溃
- ✅ **RViz 膨胀显示修复** — 修正 `pose_type=2` + 话题映射
- ✅ **实体飞行 launch** — `ego_planner_real` 包，一键启动含 RViz + URDF + TF

---

## 系统架构 / Architecture

```
Odin 传感器 ──→ /odin1/cloud_slam ──→ grid_map (占据栅格)
            ├──→ /odin1/odometry   ──→ FSM (状态机)
            └──→ /tf               ──→ RViz

上层任务 ──→ Action: ~/navigate_to_pose ──→ FSM
RViz     ──→ /move_base_simple/goal     ──→ FSM
                                    │
                              A* 全局路径 + B-spline 局部优化
                                    │
                              /drone_0_planning/bspline
                                    │
                              traj_server (100Hz)
                              位置/速度/偏航指令
                                    │
                              飞控桥接 (ENU→NED)
                                    │
                              PX4 / ArduPilot
```

---

## 包结构 / Packages

| 包 | 说明 |
|----|------|
| `ego_planner` | 核心规划器 (FSM + A* + B-spline + GridMap) |
| `ego_planner_real` | 实体飞行启动包 (launch + RViz + URDF + 文档) |
| `bspline_opt` | B-spline 轨迹优化 (L-BFGS) |
| `path_searching` | A* 全局路径搜索 |
| `plan_env` | 栅格地图 + 深度/点云融合 + 占据概率 |
| `traj_utils` | B-spline / 多项式轨迹工具 |
| `drone_detect` | 深度图中的其他无人机检测 (集群用) |
| `rosmsg_tcp_bridge` | 集群通信 TCP 桥接 |
| `quadrotor_msgs` | 自定义 ROS 消息 |

---

## 快速开始 / Quick Start

### 1. 前置依赖

```bash
# ROS 2 Humble + 基础工具
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install libpcl-dev libeigen3-dev

# 切换 DDS (推荐，FastDDS 可能卡顿)
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2. 编译

```bash
cd ~/odin_ws_3D
colcon build --packages-select ego_planner ego_planner_real
source install/setup.bash
```

### 3. 启动 (实体飞行)

```bash
# 保守首次飞行
ros2 launch ego_planner_real main.launch.py max_vel:=0.5 max_acc:=1.0 inflation:=0.2

# 开阔场地
ros2 launch ego_planner_real main.launch.py max_vel:=2.0 max_acc:=4.0 inflation:=0.12

# Z 轴锁定 (仅 XY 平面)
ros2 launch ego_planner_real main.launch.py enable_z_planning:=false
```

### 4. 发送目标

```bash
# Action 接口 (推荐)
ros2 action send_goal /drone_0_ego_planner_node/navigate_to_pose \
    ego_planner/action/NavigateToPose \
    "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}}}}"

# 机体坐标 (向前飞 3m)
ros2 action send_goal /drone_0_ego_planner_node/navigate_to_pose \
    ego_planner/action/NavigateToPose \
    "{pose: {header: {frame_id: 'body'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}}}, body_frame: true}"

# RViz "2D Goal Pose" 工具也可以
```

---

## 主要启动参数 / Key Parameters

### 运动限制 / Motion Limits

| 参数 | 默认 | 说明 |
|------|------|------|
| `max_vel` | 1.0 | 最大线速度 [m/s] |
| `max_acc` | 2.0 | 最大加速度 [m/s²] |
| `planning_horizon` | 5.0 | 规划视界 [m] |

### 障碍物 / Obstacles

| 参数 | 默认 | 说明 |
|------|------|------|
| `inflation` | 0.15 | XY 膨胀半径 [m] |
| `inflation_z` | -1.0 | Z 膨胀半径 [m]，-1=自动等于 XY |
| `self_filter_r` | 0.4 | 自身滤除半径 [m] |
| `local_inflate_r` | 3.0 | 局部膨胀范围 [m] (水平) |

### 传感器话题 / Sensor Topics

| 参数 | 默认 | 说明 |
|------|------|------|
| `odom_topic` | `/odin1/odometry` | 里程计话题 |
| `cloud_topic` | `/odin1/cloud_slam` | 点云话题 |
| `depth_topic` | `/odin1/depth_img_competetion` | 深度融合图 |
| `enable_z_planning` | `false` | Z 轴规划使能 |
| `viz_trunc_height` | 2.0 | RViz 可视化截断高度 [m] |

### 地图 / Map

| 参数 | 默认 | 说明 |
|------|------|------|
| `map_size_x` | 30.0 | 地图 X 尺寸 [m] |
| `map_size_y` | 30.0 | 地图 Y 尺寸 [m] |
| `map_size_z` | 5.0 | 地图 Z 尺寸 [m] |

---

## Action 接口 / Action API

```
/drone_0_ego_planner_node/navigate_to_pose
类型: ego_planner/action/NavigateToPose
```

| Goal | 类型 | 说明 |
|------|------|------|
| `pose` | `PoseStamped` | 目标位姿 |
| `body_frame` | `bool` | `false`=世界坐标, `true`=机体偏移 |

| Feedback (10Hz) | 类型 | 说明 |
|------|------|------|
| `current_pose` | `PoseStamped` | 当前位姿 |
| `distance_remaining` | `float32` | 剩余距离 [m] |
| `progress_percentage` | `float32` | 进度 0~100 |
| `fsm_state` | `string` | FSM 状态 |

| Result 错误码 | 值 | 说明 |
|------|----|------|
| `ERROR_NONE` | 0 | 成功 |
| `ERROR_PLANNING_FAILED` | 1 | 规划失败 |
| `ERROR_COLLISION` | 2 | 碰撞危险 |
| `ERROR_EMERGENCY_STOP` | 3 | 紧急停止 |
| `ERROR_CANCELLED` | 4 | 被取消 |
| `ERROR_NO_ODOMETRY` | 5 | 无里程计 |

---

## 坐标系 / Coordinate Frames

| | 本规划器 (ENU) | PX4/ArduPilot (NED) |
|------|------|------|
| X | 东 (East) | 北 (North) |
| Y | 北 (North) | 东 (East) |
| Z | 上 (Up) | 下 (Down) |

飞控桥接需转换: `NED_x=ENU_y, NED_y=ENU_x, NED_z=-ENU_z`

---

## 输出接口 / Output

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `~/position_setpoint` | `PoseStamped` | 100Hz | 位置指令 |
| `~/velocity_setpoint` | `TwistStamped` | 100Hz | 速度指令 |
| `~/yaw_setpoint` | `Float32` | 100Hz | 偏航角 |

---

## License

**GPL v3** — 继承自 [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)

本仓库是一个修改版本 (modified version)，修改内容包括但不限于：ROS 2 Action 接口、Z 轴规划使能、独立 Z 膨胀、Mid-360 适配、时间源修复。

This is a modified version of the original ego-planner-swarm. Modifications include: ROS 2 Action interface, Z-axis planning toggle, independent Z inflation, Mid-360 LiDAR support, clock sync fixes.

---

## 致谢 / Acknowledgments

- 原始 EGO-Planner 论文: [Zhou et al., "EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors"](https://arxiv.org/abs/2008.08837)
- 上游仓库: [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
