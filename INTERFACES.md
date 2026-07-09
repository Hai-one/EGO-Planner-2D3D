# EGO-Planner 实体飞行接口文档

---

## 输入接口

> **Topic 目标接口 (`/ego_planner/goal`, `~/goal_body`) 已移除。请统一使用 Action 接口。**

### 1. Action 导航接口

标准 ROS 2 ==Action== 接口，支持反馈、取消和结果报告。**推荐上层任务系统优先使用此接口**。

| 属性           | 值                                             |
| -------------- | ---------------------------------------------- |
| ==Action== | `/drone_0_ego_planner_node/navigate_to_pose` |
| 类型           | `ego_planner/action/NavigateToPose`          |

**Goal 参数:**

| 字段           | 类型                          | 说明                                            |
| -------------- | ----------------------------- | ----------------------------------------------- |
| `pose`       | `geometry_msgs/PoseStamped` | 目标位姿                                        |
| `body_frame` | `bool`                      | `false`=世界坐标(默认), `true`=机体坐标偏移 |

**Feedback (10Hz):**

| 字段                    | 类型            | 说明                                                                               |
| ----------------------- | --------------- | ---------------------------------------------------------------------------------- |
| `current_pose`        | `PoseStamped` | 无人机当前位姿                                                                     |
| `distance_remaining`  | `float32`     | 到目标点的剩余距离 [m]                                                             |
| `progress_percentage` | `float32`     | 进度百分比 0~100                                                                   |
| `fsm_state`           | `string`      | 当前 FSM 状态 (INIT/WAIT_TARGET/GEN_NEW_TRAJ/REPLAN_TRAJ/EXEC_TRAJ/EMERGENCY_STOP) |

**Result 错误码:**

| 常量                      | 值 | 说明                   |
| ------------------------- | -- | ---------------------- |
| `ERROR_NONE`            | 0  | 成功到达目标           |
| `ERROR_PLANNING_FAILED` | 1  | 规划失败 (重试超限)    |
| `ERROR_COLLISION`       | 2  | 碰撞危险               |
| `ERROR_EMERGENCY_STOP`  | 3  | 紧急停止               |
| `ERROR_CANCELLED`       | 4  | 被取消                 |
| `ERROR_NO_ODOMETRY`     | 5  | 无里程计 (Goal 被拒绝) |

```bash
# 发送世界坐标目标
ros2 action send_goal /drone_0_ego_planner_node/navigate_to_pose \
    ego_planner/action/NavigateToPose \
    "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 3.0, y: 2.0, z: 1.0}}}}"

# 发送机体坐标目标 (向前飞 3 米)
ros2 action send_goal /drone_0_ego_planner_node/navigate_to_pose \
    ego_planner/action/NavigateToPose \
    "{pose: {header: {frame_id: 'body'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}}}, body_frame: true}"

# 取消当前导航
ros2 action send_goal --cancel /drone_0_ego_planner_node/navigate_to_pose
```

### 2. RViz 2D Nav Goal (兼容)

RViz 工具栏 → "2D Goal Pose" → 在地图上点击。

| 属性 | 值                                |
| ---- | --------------------------------- |
| 话题 | `/move_base_simple/goal`        |
| 类型 | `geometry_msgs/msg/PoseStamped` |

---

## 输出接口

### 控制指令 (traj_server, 100Hz)

| 话题                                       | 类型                | 帧   | 内容                                         |
| ------------------------------------------ | ------------------- | ---- | -------------------------------------------- |
| `/drone_0_traj_server/position_setpoint` | `PoseStamped`     | odom | 位置 (x,y,z) + 偏航四元数                    |
| `/drone_0_traj_server/velocity_setpoint` | `TwistStamped`    | odom | 线速度 (vx,vy,vz) + 偏航角速率               |
| `/drone_0_traj_server/yaw_setpoint`      | `Float32`         | —   | 偏航角 [rad]                                 |
| `/position_cmd`                          | `PositionCommand` | odom | 完整指令 (位置+速度+加速度+偏航, 兼容旧接口) |

飞控桥接按需订阅任意一个即可。位置模式用 `position_setpoint`，速度模式用 `velocity_setpoint`。

### 状态反馈

| 话题                                       | 类型                        | 说明                                                          |
| ------------------------------------------ | --------------------------- | ------------------------------------------------------------- |
| ==Action== Feedback                    | `NavigateToPose.Feedback` | **推荐**: 通过 ==Action== 获取实时进度/距离/FSM状态 |
| `/drone_0_ego_planner_node/goal_reached` | `Bool`                    | `false`=飞行中, `true`=已到达                             |
| `/drone_0_ego_planner_node/current_pose` | `PoseStamped`             | 规划器收到的当前里程计位姿 (监控用)                           |
| `/drone_0_ego_planner_node/replan_start` | `PoseStamped`             | 每次重规划的起点 (诊断用)                                     |

### 可视化

| 话题                                         | 类型        | 说明                     |
| -------------------------------------------- | ----------- | ------------------------ |
| `/drone_0_plan_vis/goal_point`             | Marker      | 目标点                   |
| `/drone_0_plan_vis/global_list`            | Marker      | A* 全局路径 (青色)       |
| `/drone_0_plan_vis/optimal_list`           | Marker      | B-spline 优化轨迹 (红色) |
| `/drone_0_grid/grid_map/occupancy_inflate` | PointCloud2 | 膨胀后的占据栅格         |

---

## 启动参数

### 运动限制

| 参数                 | 默认 | 范围     | 说明                                             |
| -------------------- | ---- | -------- | ------------------------------------------------ |
| `max_vel`          | 1.0  | 0.3~3.0  | 最大线速度 [m/s]。首次飞行 ≤1.0, 稳定后逐步提高 |
| `max_acc`          | 2.0  | 0.5~6.0  | 最大加速度 [m/s²]。越小轨迹越平滑, 越大越激进   |
| `planning_horizon` | 5.0  | 3.0~10.0 | 局部规划视界 [m]。越大路径越长但计算量越大       |

### 障碍物处理

| 参数                | 默认 | 范围      | 说明                                                                                         |
| ------------------- | ---- | --------- | -------------------------------------------------------------------------------------------- |
| `inflation`       | 0.15 | 0.05~0.40 | **障碍物膨胀半径 [m]**。每个点云点向外膨胀此距离形成禁飞区。越大越安全但通道可能被堵死 |
| `self_filter_r`   | 0.3  | 0.1~0.6   | **自身滤除半径 [m]**。无人机本体也会被扫描到, 此范围内的点云直接忽略                   |
| `local_inflate_r` | 3.0  | 1.0~6.0   | **局部膨胀范围 [m]** (水平)。超过此距离的障碍物只标记不膨胀, 减少远处噪点影响          |

### 实时性

| 参数                   | 默认 | 说明                                             |
| ---------------------- | ---- | ------------------------------------------------ |
| `thresh_replan_time` | 0.15 | 重规划周期 [s]。150ms 保证对突现障碍物的快速反应 |
| `emergency_time`     | 0.3  | 紧急制动时间 [s]。发现碰撞危险后多快触发急停     |

### 地图

| 参数               | 默认                             | 说明         |
| ------------------ | -------------------------------- | ------------ |
| `map_size_x/y/z` | 30/30/5                          | 地图尺寸 [m] |
| `odom_topic`     | `/odin1/odometry`              | 里程计话题   |
| `cloud_topic`    | `/odin1/cloud_slam`            | 点云话题     |
| `depth_topic`    | `/odin1/depth_img_competetion` | 深度融合图   |

---

## 坐标系

|            | 本规划器                 | PX4/ArduPilot            |
| ---------- | ------------------------ | ------------------------ |
| 坐标系     | **ENU** (东-北-上) | **NED** (北-东-下) |
| 偏航 0°   | +X 轴 (东)               | +X 轴 (北)               |
| 高度正方向 | +Z                       | -Z                       |

飞控桥接需转换: `NED_x=ENU_y, NED_y=ENU_x, NED_z=-ENU_z, yaw_ned=yaw_enu-π/2`

---

## 启动示例

```bash
# 基础启动
ros2 launch ego_planner_real main.launch.py

# 保守飞行 (首次测试)
ros2 launch ego_planner_real main.launch.py max_vel:=0.5 max_acc:=1.0 inflation:=0.2

# 开阔场地 (放开限制)
ros2 launch ego_planner_real main.launch.py max_vel:=2.0 max_acc:=4.0 inflation:=0.12

# 狭窄环境 (增大膨胀 + 缩短视界)
ros2 launch ego_planner_real main.launch.py inflation:=0.25 planning_horizon:=3.0 self_filter_r:=0.4

# 用高频里程计
ros2 launch ego_planner_real main.launch.py odom_topic:=/odin1/odometry_highfreq

# 指定地图大小
ros2 launch ego_planner_real main.launch.py map_size_x:=50.0 map_size_y:=50.0 map_size_z:=8.0
```

---

## 控制流程

```
Odin 传感器 ──→ /odin1/cloud_slam ──→ grid_map (占据栅格)
            ──→ /odin1/odometry   ──→ FSM (里程计)
            ──→ /tf               ──→ RViz (TF 树)

上层任务 ──→ Action: ~/navigate_to_pose ──→ FSM
RViz     ──→ Topic: /move_base_simple/goal ──→ FSM    ← MANUAL_TARGET 模式
                                    │
                              A* 全局路径 + B-spline 局部优化
                                    │
                              /drone_0_planning/bspline
                                    │
                              traj_server (100Hz 采样)
                                    │
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
            position_setpoint  velocity_setpoint  yaw_setpoint
            (PoseStamped)      (TwistStamped)     (Float32)
                    │
                    ▼
              飞控桥接 (ENU→NED)
                    │
                    ▼
              PX4/ArduPilot
```
