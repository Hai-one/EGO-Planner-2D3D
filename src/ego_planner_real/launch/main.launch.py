"""
============================================================================
  EGO-Planner 实体飞行 — 一键启动文件
  Odin 驱动 → 栅格地图 → 路径规划 → 轨迹服务器 → RViz 可视化
============================================================================

用法:
    ros2 launch ego_planner_real main.launch.py
    ros2 launch ego_planner_real main.launch.py max_vel:=1.5 inflation:=0.2

输入接口 (发送目标点):
    ros2 action send_goal /drone_0_ego_planner_node/navigate_to_pose \
        ego_planner/action/NavigateToPose \
        "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 1.5}}}}"

输出接口 (飞控桥接):
    /drone_0_traj_server/position_setpoint   PoseStamped   20HZ
    /drone_0_traj_server/velocity_setpoint   TwistStamped  20HZ
    /drone_0_traj_server/yaw_setpoint        Float32       20HZ
    /drone_0_traj_server/cmd_vel             Twist         20HZ    (仿move_base)
    /drone_0_ego_planner_node/goal_reached   Bool          到达通知
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ================================================================
    #  传感器话题 (Odin 驱动发布)
    # ================================================================
    odom_topic   = LaunchConfiguration('odom_topic',
                     default='/Odometry')            # 里程计话题 [Odin SLAM odom]
    cloud_topic  = LaunchConfiguration('cloud_topic',
                     default='/cloud_registered')          # SLAM 点云话题 [PointCloud2]
    # depth_topic  = LaunchConfiguration('depth_topic',
    #                  default='/odin1/depth_img_competetion') # 深度融合图 [Image]

    # ================================================================
    #  运动限制 — 初次飞行保持保守, 稳定后逐步放开
    # ================================================================
    drone_id          = LaunchConfiguration('drone_id',           default='0')
    max_vel           = LaunchConfiguration('max_vel',            default='1.0')   # 最大线速度 [m/s], 建议 0.5~2.0
    max_acc           = LaunchConfiguration('max_acc',            default='2.0')#default 2.0   # 最大加速度 [m/s²], 建议 1.0~4.0
    planning_horizon  = LaunchConfiguration('planning_horizon',   default='3.0')   # 规划视界 [m], 局部路径长度, 3~10m

    # ================================================================
    #  栅格地图 — 决定避障行为
    # ================================================================
    map_size_x   = LaunchConfiguration('map_size_x',  default='30.0')   # 地图 X 尺寸 [m]
    map_size_y   = LaunchConfiguration('map_size_y',  default='30.0')   # 地图 Y 尺寸 [m]
    map_size_z   = LaunchConfiguration('map_size_z',  default='5.0')    # 地图 Z 尺寸 [m] (高度)

    # ── 障碍物膨胀 ──
    inflation       = LaunchConfiguration('inflation',
                        default='0.35')   # 膨胀半径 [m] (XY), 每个障碍物点向外膨胀此距离
                                          # 越大越安全但可能堵死通道, 0.10~0.30
    inflation_z     = LaunchConfiguration('inflation_z',
                        default='-1.0')   # Z 轴膨胀半径 [m], -1 表示自动等于 inflation
    self_filter_r   = LaunchConfiguration('self_filter_r',
                        default='0.20')    # 自身滤除半径 [m], 此范围内的点云被忽略
                                          # 防止无人机本体被当作障碍物, 0.2~0.5
    local_inflate_r = LaunchConfiguration('local_inflate_r',
                        default='2.0')    # 局部膨胀范围 [m](水平), 超出此范围的障碍物不膨胀
                                          # 减少远处噪点干扰, 2.0~5.0

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # ── 可视化 ──
    viz_trunc_height = LaunchConfiguration('viz_trunc_height', default='2.0')

    # ── Z 轴规划 ──
    enable_z_planning = LaunchConfiguration('enable_z_planning', default='true')

    # ================================================================
    #  参数声明
    # ================================================================
    args = [
        DeclareLaunchArgument('odom_topic',       default_value=odom_topic,
            description='里程计话题 [/odin1/odometry 或 /odin1/odometry_highfreq]'),
        DeclareLaunchArgument('cloud_topic',      default_value=cloud_topic,
            description='SLAM 点云话题 [/odin1/cloud_slam]'),
        # DeclareLaunchArgument('depth_topic',      default_value=depth_topic,
        #     description='深度融合图话题'),
        DeclareLaunchArgument('drone_id',         default_value=drone_id),
        DeclareLaunchArgument('max_vel',          default_value=max_vel,
            description='最大线速度 [m/s], 首次飞行建议 0.5~1.0'),
        DeclareLaunchArgument('max_acc',          default_value=max_acc,
            description='最大加速度 [m/s²], 首次飞行建议 1.0~2.0'),
        DeclareLaunchArgument('planning_horizon', default_value=planning_horizon,
            description='规划视界 [m], 每次重规划的前瞻距离'),
        DeclareLaunchArgument('map_size_x',       default_value=map_size_x),
        DeclareLaunchArgument('map_size_y',       default_value=map_size_y),
        DeclareLaunchArgument('map_size_z',       default_value=map_size_z),
        DeclareLaunchArgument('use_rviz',         default_value=use_rviz),
        DeclareLaunchArgument('inflation',        default_value=inflation,
            description='障碍物膨胀半径 [m](XY), 0.10~0.30'),
        DeclareLaunchArgument('inflation_z',      default_value=inflation_z,
            description='Z轴膨胀半径 [m], -1=自动等于XY膨胀'),
        DeclareLaunchArgument('self_filter_r',    default_value=self_filter_r,
            description='自身滤除半径 [m], 忽略无人机本体的点云'),
        DeclareLaunchArgument('local_inflate_r',  default_value=local_inflate_r,
            description='局部膨胀范围 [m](水平), 超出不膨胀'),
        DeclareLaunchArgument('enable_z_planning', default_value=enable_z_planning,
            description='Z轴规划使能 [true=3D规划, false=锁定Z仅XY]'),
        DeclareLaunchArgument('viz_trunc_height',  default_value=viz_trunc_height,
            description='可视化截断高度 [m], 超过此高度的障碍物不显示'),
    ]

    # ================================================================
    #  TF: odom → body (兜底)
    #  Odin 连上后会发布动态 TF 覆盖此静态值, 模型跟随无人机移动
    # ================================================================
    fallback_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0','0','0','0','0','0', 'odom', 'body'],
        output='screen',
    )

    # ================================================================
    #  URDF 模型 (robot_state_publisher)
    #  把 robot.urdf 里的固定关节发布为 TF: body → arms → motors → props
    # ================================================================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(os.path.join(
                get_package_share_directory('ego_planner_real'),
                'config', 'robot.urdf')).read(),
        }],
    )

    # ================================================================
    #  Odin ROS Driver (传感器驱动, 发布 odometry + 点云 + TF)
    # ================================================================
    # odin_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('odin_ros_driver'),
    #         'launch', 'odin1_ros2.launch.py')),
    # )

    # ================================================================
    #  EGO-Planner 主节点 — FSM + A* + B-spline 优化
    # ================================================================
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        # ── 话题映射: 内部话题 → 全局绝对话题 ──
        # 注意: 相对路径在 ROS 2 中解析行为不确定, 统一使用 / 前缀
        remappings=[
            ('odom_world',                     odom_topic),
            ('grid_map/odom',                  odom_topic),
            ('grid_map/cloud',                 cloud_topic),
            # ('grid_map/depth',                 depth_topic),
            ('planning/bspline',               ['/drone_', drone_id, '_planning/bspline']),
            ('planning/data_display',          ['/drone_', drone_id, '_planning/data_display']),
            ('goal_point',                     ['/drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list',                    ['/drone_', drone_id, '_plan_vis/global_list']),
            ('init_list',                      ['/drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list',                   ['/drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list',                    ['/drone_', drone_id, '_plan_vis/a_star_list']),
            ('grid_map/occupancy_inflate',     ['/drone_', drone_id, '_grid/grid_map/occupancy_inflate']),
        ],
        parameters=[
            # ═══════════════ FSM 状态机 ═══════════════
            {'fsm/flight_type': 1},                    # 1=手动目标, 2=预设航点
            {'fsm/thresh_replan_time': 0.15},          # 重规划周期 [s], 150ms 实时反应
            {'fsm/thresh_no_replan_meter': 0.5},       # 接近目标阈值 [m], 小于此距离跳过重规划
            {'fsm/planning_horizon': planning_horizon}, # 局部规划视界 [m]
            {'fsm/planning_horizen_time': 3.0},        # 时间维度的规划视界 [s]
            {'fsm/emergency_time': 0.3},               # 紧急制动触发时间 [s], 发现障碍物后多久急停
            {'fsm/realworld_experiment': True},         # 实体模式 (等待 goal + odom 才触发)
            {'fsm/fail_safe': True},                   # 失效保护 (深度丢失 → EMERGENCY_STOP)
            {'fsm/enable_z_planning': enable_z_planning}, # Z轴规划使能 (false=仅XY平面)
            {'fsm/waypoint_num': 0},                   # 预设航点数 (flight_type=2 时生效)

            # ═══════════════ 栅格地图 ═══════════════
            {'grid_map/frame_id': 'odom'},             # 地图坐标系 (对齐 Odin)
            {'grid_map/resolution': 0.1},              # 栅格分辨率 [m], 越小越精细但计算量越大
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 5.5},    # 局部更新范围 X [m]
            {'grid_map/local_update_range_y': 5.5},    # 局部更新范围 Y [m]
            {'grid_map/local_update_range_z': 4.5},    # 局部更新范围 Z [m]
            # ── 障碍物处理 ──
            {'grid_map/obstacles_inflation': inflation},        # 膨胀半径 [m] (XY)
            {'grid_map/obstacles_inflation_z': inflation_z},  # 膨胀半径 Z [m], -1=自动等于XY
            {'grid_map/self_filter_radius': self_filter_r},    # 自身滤除半径 [m]
            {'grid_map/local_inflation_range': local_inflate_r},# 局部膨胀范围 [m](水平)
            {'grid_map/local_map_margin': 10},          # 局部地图边界余量
            {'grid_map/ground_height': -0.01},          # 地面高度 [m], 低于此值忽略
            # ── 相机内参 (Odin 默认, 深度融合用) ──
            {'grid_map/cx': 321.04638671875},
            {'grid_map/cy': 243.44969177246094},
            {'grid_map/fx': 387.229248046875},
            {'grid_map/fy': 387.229248046875},
            # ── 深度滤波 ──
            {'grid_map/use_depth_filter': True},         # 启用深度融合滤波
            {'grid_map/depth_filter_tolerance': 0.15},   # 深度滤波容差 [m]
            {'grid_map/depth_filter_maxdist': 5.0},      # 深度滤波最大距离 [m]
            {'grid_map/depth_filter_mindist': 0.2},      # 深度滤波最小距离 [m]
            {'grid_map/depth_filter_margin': 2},         # 深度滤波边界
            {'grid_map/k_depth_scaling_factor': 1000.0}, # 深度缩放因子
            {'grid_map/skip_pixel': 2},                  # 跳像素 (加速)
            # ── 占据概率 ──
            {'grid_map/p_hit': 0.65}, {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12}, {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1}, {'grid_map/max_ray_length': 4.5},
            # ── 虚拟天花板 ──
            {'grid_map/virtual_ceil_height': 4.0},       # 虚拟天花板高度 [m], 限制飞行高度
            {'grid_map/visualization_truncate_height': viz_trunc_height},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 2},                   # 2=Odometry (Odin 发布的是 Odometry, 非 PoseStamped)

            # ═══════════════ 运动限制 ═══════════════
            {'manager/max_vel': max_vel},                # 最大线速度 [m/s]
            {'manager/max_acc': max_acc},                # 最大加速度 [m/s²]
            {'manager/max_jerk': 4.0},                   # 最大 jerk [m/s³]
            {'manager/control_points_distance': 0.4},    # B-spline 控制点间距 [m]
            {'manager/feasibility_tolerance': 0.05},     # 可行性容差 (允许超限比例)
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': True},     # 生成多条候选轨迹选最优
            {'manager/drone_id': drone_id},

            # ═══════════════ B-spline 优化 ═══════════════
            {'optimization/lambda_smooth': 1.0},         # 平滑代价权重
            {'optimization/lambda_collision': 0.5},      # 碰撞代价权重 (越大越远离障碍物)
            {'optimization/lambda_feasibility': 0.1},    # 可行性代价权重
            {'optimization/lambda_fitness': 1.0},        # 适应度代价权重
            {'optimization/dist0': 0.5},                 # 碰撞距离阈值 [m]
            {'optimization/swarm_clearance': 0.5},       # 集群安全距离 [m] (单机忽略)
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},                # 极限比例 (允许短时间超限)
            {'prediction/obj_num': 0},                   # 动态障碍物数量 (实体飞行=0)
        ]
    )

    # ================================================================
    #  轨迹服务器 — B-spline → 位置/速度/偏航指令 (100Hz)
    #  输出: ~/position_setpoint, ~/velocity_setpoint, ~/yaw_setpoint
    # ================================================================
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('planning/bspline', ['/drone_', drone_id, '_planning/bspline']),
            ('odom_world', odom_topic),                         # 用于偏离检测
        ],
        parameters=[
            {'traj_server/time_forward': 1.0},                  # 偏航前瞻时间 [s]
            {'traj_server/max_deviation': 1.0},                 # 最大偏离 [m], 超过则悬停+平滑收敛
            {'traj_server/pos_gain_x': 5.0},                    # 位置增益 X [飞控用]
            {'traj_server/pos_gain_y': 5.0},
            {'traj_server/pos_gain_z': 5.0},
            {'traj_server/vel_gain_x': 2.0},                    # 速度增益 X [飞控用]
            {'traj_server/vel_gain_y': 2.0},
            {'traj_server/vel_gain_z': 2.0},
        ]
    )

    # ================================================================
    #  RViz — 加载 real_world.rviz 配置
    # ================================================================
    rviz_config = os.path.join(
        get_package_share_directory('ego_planner_real'),
        'config', 'real_world.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=['-d', rviz_config],
    )

    # ================================================================
    #  组装
    # ================================================================
    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)
    # ld.add_action(odin_launch)
    ld.add_action(fallback_tf)
    ld.add_action(robot_state_pub)
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)
    ld.add_action(rviz_node)

    return ld
