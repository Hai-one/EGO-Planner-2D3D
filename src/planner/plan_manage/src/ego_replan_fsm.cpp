
#include <ego_planner/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);

    node_->declare_parameter("fsm/enable_z_planning", true);
    node_->get_parameter("fsm/enable_z_planning", enable_z_planning_);

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);
    goal_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>("~/goal_reached", 10);
    current_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);
    replan_start_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("~/replan_start", 10);

    // ── Action Server ──────────────────────────────────────
    // 提供标准的 ROS 2 Action 导航接口, 支持反馈/取消/结果报告。
    // 话题: ~/navigate_to_pose (ego_planner/action/NavigateToPose)
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        node_,
        "~/navigate_to_pose",
        std::bind(&EGOReplanFSM::actionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&EGOReplanFSM::actionHandleCancel, this, std::placeholders::_1),
        std::bind(&EGOReplanFSM::actionHandleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "[EGO Planner] Action server ready: ~/navigate_to_pose");

    // 初始化反馈节流时钟 (须用 node_->now() 保证时钟类型一致)
    last_feedback_time_ = node_->now();

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    // Z 轴锁定: 关闭 Z 规划时, 目标高度 = 当前高度
    Eigen::Vector3d end_wp = next_wp;
    if (!enable_z_planning_)
      end_wp.z() = odom_pos_.z();

    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = end_wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          rclcpp::spin_some(node_);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    // ── RViz 2D Nav Goal 回调 ──────────────────────────────
    // 话题: /move_base_simple/goal  (仅 MANUAL_TARGET 模式下启用)
    if (msg->pose.position.z < -0.1)
      return;

    // 若 Action 活跃, 先 abort
    {
      bool need_abort = false;
      {
        std::lock_guard<std::mutex> lock(action_mutex_);
        need_abort = (action_active_ && active_goal_handle_ && active_goal_handle_->is_active());
      }
      if (need_abort)
        setActionResult(false, NavigateToPose::Result::ERROR_CANCELLED,
                        "Goal overridden by RViz 2D Nav Goal.");
    }

    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;

    // 发布当前位姿 (调试/监控用)
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "odom";
    pose_msg.pose = msg->pose.pose;
    current_pose_pub_->publish(pose_msg);
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto msg_time = rclcpp::Time(msg->start_time, steady_clock.get_clock_type());
    if (abs((steady_clock.now() - msg_time).seconds()) > 0.25)
    {
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (steady_clock.now() - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // 计算路径持续时间
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;

    // 进入 GEN_NEW_TRAJ 或 REPLAN_TRAJ → 标记正在执行 (还未到达)
    if (new_state == GEN_NEW_TRAJ || new_state == REPLAN_TRAJ)
    {
      std_msgs::msg::Bool reached;
      reached.data = false;
      goal_reached_pub_->publish(reached);
    }

    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage

    // ── Action 反馈 (每周期发布) ──
    publishActionFeedback();

    // ── Action 取消检查 ──
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (action_active_ && active_goal_handle_ && active_goal_handle_->is_canceling())
      {
        RCLCPP_WARN(node_->get_logger(), "[Action] Cancel in progress — triggering emergency stop.");
        emergency_from_action_cancel_ = true;
        changeFSMExecState(EMERGENCY_STOP, "ACTION_CANCEL");
      }
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        // 连续失败达到上限 → 放弃并报告失败
        if (continously_called_times_ >= max_planning_retries_)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "[Action] Max planning retries (%d) exceeded. Aborting.",
                       max_planning_retries_);
          setActionResult(false, NavigateToPose::Result::ERROR_PLANNING_FAILED,
                          "Planning failed after max retries (" +
                              std::to_string(max_planning_retries_) + ").");
          have_target_ = false;
          have_trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          // ── 目标到达 ──
          {
            std_msgs::msg::Bool reached;
            reached.data = true;
            goal_reached_pub_->publish(reached);
            RCLCPP_INFO(node_->get_logger(), "[EGO Planner] Goal reached: (%.2f, %.2f, %.2f)",
                        end_pt_(0), end_pt_(1), end_pt_(2));
            // Action 成功 (不在锁内调用)
            setActionResult(true, NavigateToPose::Result::ERROR_NONE,
                            string("Goal reached: (") +
                                std::to_string(end_pt_(0)) + ", " +
                                std::to_string(end_pt_(1)) + ", " +
                                std::to_string(end_pt_(2)) + ")");
          }

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
        // 报告 Action 结果: 区分用户取消 vs 真实紧急停止
        if (emergency_from_action_cancel_)
        {
          setActionResult(false, NavigateToPose::Result::ERROR_CANCELLED,
                          "Navigation cancelled by user.");
          emergency_from_action_cancel_ = false;
        }
        else
        {
          setActionResult(false, NavigateToPose::Result::ERROR_EMERGENCY_STOP,
                          "Emergency stop triggered.");
        }
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    start_pt_ = odom_pos_;
    // 不依赖飞机当前速度，直接用目标方向 × max_vel
    {
      Eigen::Vector3d diff = end_pt_ - start_pt_;
      if (diff.norm() > 0.01)
        start_vel_ = diff.normalized() * planner_manager_->pp_.max_vel_;
      else
        start_vel_ = odom_vel_;  // fallback: 目标太近
    }
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = node_->now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    // 始终以无人机真实位置为起点，速度不依赖当前状态
    start_pt_ = odom_pos_;
    {
      Eigen::Vector3d diff = end_pt_ - start_pt_;
      if (diff.norm() > 0.01)
        start_vel_ = diff.normalized() * planner_manager_->pp_.max_vel_;
      else
        start_vel_ = odom_vel_;
    }
    start_acc_ = Eigen::Vector3d::Zero();

    // 强制多项式初始化 — 不用旧轨迹的点, 从 odom_pos_ 重新生成
    bool success = callReboundReplan(true, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    // ── 无人机未明显移动时跳过碰撞检测，避免原地反复重规划 ──
    {
      double dist_from_start = (odom_pos_ - info->start_pos_).norm();
      if (dist_from_start < 0.3)  // 移动不足 0.3m，认为尚未起飞
        return;
    }

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_cur = (node_->now() - info->start_time_).seconds();

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = node_->now().seconds();

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      // Z 轴锁定: 关闭 Z 规划时, 所有控制点 Z 统一到当前高度
      if (!enable_z_planning_)
      {
        Eigen::MatrixXd pos_pts_z = info->position_traj_.getControlPoint();
        for (int i = 0; i < pos_pts_z.cols(); ++i)
          pos_pts_z(2, i) = odom_pos_.z();
        // 用修正后的控制点重建 B-spline (保持原速度和加速度梯度)
        info->position_traj_ = UniformBspline(pos_pts_z, 3, info->position_traj_.getTimeSum() / (pos_pts_z.cols() - 3));
        info->velocity_traj_ = info->position_traj_.getDerivative();
        info->acceleration_traj_ = info->velocity_traj_.getDerivative();
      }

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      /* diagnostic: publish replan start point */
      {
        geometry_msgs::msg::PoseStamped diag;
        diag.header.stamp = node_->now();
        diag.header.frame_id = "odom";
        diag.pose.position.x = start_pt_(0);
        diag.pose.position.y = start_pt_(1);
        diag.pose.position.z = start_pt_(2);
        replan_start_pub_->publish(diag);
      }

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }

    // Z 轴锁定: 关闭 Z 规划时, 局部目标高度 = 当前高度
    if (!enable_z_planning_)
      local_target_pt_.z() = odom_pos_.z();
  }

  // ────────────────────────────────────────────────────────
  // Action Server 实现
  // ────────────────────────────────────────────────────────

  rclcpp_action::GoalResponse EGOReplanFSM::actionHandleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    (void)uuid;

    // 无里程计 → 拒绝
    if (!have_odom_)
    {
      RCLCPP_WARN(node_->get_logger(), "[Action] Goal rejected: no odometry.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 世界坐标 z < -0.1 → 拒绝 (body_frame 目标允许 z 为任意值)
    if (!goal->body_frame && goal->pose.pose.position.z < -0.1)
    {
      RCLCPP_WARN(node_->get_logger(), "[Action] Goal rejected: z < -0.1 (below ground).");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(node_->get_logger(), "[Action] Goal accepted.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse EGOReplanFSM::actionHandleCancel(
      const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(node_->get_logger(), "[Action] Cancel requested — will stop at next FSM cycle.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void EGOReplanFSM::actionHandleAccepted(
      const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "[Action] Goal accepted, starting execution.");

    // 如果已有活跃 Action, 先 abort 旧目标
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (active_goal_handle_ && active_goal_handle_->is_active())
      {
        auto result = std::make_shared<NavigateToPose::Result>();
        result->success = false;
        result->error_code = NavigateToPose::Result::ERROR_CANCELLED;
        result->message = "Replaced by a new action goal.";
        active_goal_handle_->abort(result);
        RCLCPP_WARN(node_->get_logger(), "[Action] Previous goal aborted (replaced).");
      }
      active_goal_handle_ = goal_handle;
      action_active_ = true;
      action_start_time_ = node_->now();
    }

    // 坐标转换: body_frame → world, 或直接提取世界坐标
    Eigen::Vector3d end_wp;
    if (goal_handle->get_goal()->body_frame)
    {
      end_wp = actionGoalToWorld(*goal_handle->get_goal());
    }
    else
    {
      end_wp(0) = goal_handle->get_goal()->pose.pose.position.x;
      end_wp(1) = goal_handle->get_goal()->pose.pose.position.y;
      end_wp(2) = goal_handle->get_goal()->pose.pose.position.z;
      if (fabs(end_wp(2)) < 0.01)
        end_wp(2) = 1.0; // 默认高度 1m
    }

    action_goal_pose_ = goal_handle->get_goal()->pose;

    RCLCPP_INFO(node_->get_logger(), "[Action] Target: (%.2f, %.2f, %.2f) body_frame=%d",
                end_wp(0), end_wp(1), end_wp(2), goal_handle->get_goal()->body_frame);

    have_trigger_ = true;
    init_pt_ = odom_pos_;
    planNextWaypoint(end_wp);
  }

  Eigen::Vector3d EGOReplanFSM::actionGoalToWorld(const NavigateToPose::Goal &goal)
  {
    // 与 goalBodyCallback 完全一致的机体→世界坐标转换
    Eigen::Vector3d body_offset(goal.pose.pose.position.x,
                                goal.pose.pose.position.y,
                                goal.pose.pose.position.z);

    // 仅偏航旋转 (忽略 roll/pitch)
    Eigen::AngleAxisd yaw_rot(
        atan2(2.0 * (odom_orient_.w() * odom_orient_.z() +
                     odom_orient_.x() * odom_orient_.y()),
              1.0 - 2.0 * (odom_orient_.y() * odom_orient_.y() +
                           odom_orient_.z() * odom_orient_.z())),
        Eigen::Vector3d::UnitZ());
    Eigen::Vector3d world_offset = yaw_rot * body_offset;
    world_offset.z() = body_offset.z();

    Eigen::Vector3d end_wp = odom_pos_ + world_offset;
    if (end_wp.z() < 0.1)
      end_wp.z() = 0.5;
    return end_wp;
  }

  void EGOReplanFSM::publishActionFeedback()
  {
    // 节流: 10Hz (100ms 间隔)
    auto now = node_->now();
    if ((now - last_feedback_time_).seconds() < 0.1)
      return;
    last_feedback_time_ = now;

    std::lock_guard<std::mutex> lock(action_mutex_);

    if (!action_active_ || !active_goal_handle_ || !active_goal_handle_->is_active())
      return;

    auto feedback = std::make_shared<NavigateToPose::Feedback>();

    // 当前位置
    feedback->current_pose.header.stamp = node_->now();
    feedback->current_pose.header.frame_id = "odom";
    feedback->current_pose.pose.position.x = odom_pos_(0);
    feedback->current_pose.pose.position.y = odom_pos_(1);
    feedback->current_pose.pose.position.z = odom_pos_(2);
    feedback->current_pose.pose.orientation.w = odom_orient_.w();
    feedback->current_pose.pose.orientation.x = odom_orient_.x();
    feedback->current_pose.pose.orientation.y = odom_orient_.y();
    feedback->current_pose.pose.orientation.z = odom_orient_.z();

    // 剩余距离
    double dist = (end_pt_ - odom_pos_).norm();
    feedback->distance_remaining = static_cast<float>(dist);

    // 进度百分比 (基于全局轨迹时间)
    if (planner_manager_->global_data_.global_duration_ > 0)
    {
      double progress = planner_manager_->global_data_.last_progress_time_ /
                        planner_manager_->global_data_.global_duration_;
      feedback->progress_percentage = static_cast<float>(std::min(progress, 1.0) * 100.0);
    }
    else
    {
      feedback->progress_percentage = 0.0f;
    }

    // FSM 状态字符串
    static const char *state_names[] = {
        "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
        "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int state_idx = static_cast<int>(exec_state_);
    if (state_idx >= 0 && state_idx < 7)
      feedback->fsm_state = state_names[state_idx];
    else
      feedback->fsm_state = "UNKNOWN";

    active_goal_handle_->publish_feedback(feedback);
  }

  void EGOReplanFSM::setActionResult(bool success, uint8_t error_code, const std::string &message)
  {
    std::lock_guard<std::mutex> lock(action_mutex_);

    if (!action_active_ || !active_goal_handle_)
      return;

    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = success;
    result->error_code = error_code;
    result->message = message;

    if (!active_goal_handle_->is_active())
    {
      action_active_ = false;
      active_goal_handle_.reset();
      return;
    }

    if (error_code == NavigateToPose::Result::ERROR_CANCELLED)
      active_goal_handle_->canceled(result);
    else if (success)
      active_goal_handle_->succeed(result);
    else
      active_goal_handle_->abort(result);

    RCLCPP_INFO(node_->get_logger(), "[Action] Result: success=%d code=%d msg=%s",
                success, error_code, message.c_str());

    action_active_ = false;
    active_goal_handle_.reset();
  }

} // namespace ego_planner
