#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <rclcpp/rclcpp.hpp>

// ── 标准化输出接口 ──────────────────────────────────────────
// 除了自定义 PositionCommand，同时发布标准 ROS2 消息，
// 方便任意飞控桥接节点直接订阅，无需依赖自定义消息类型。
//
// 话题:
//   ~/position_setpoint  (geometry_msgs::msg::PoseStamped)  – 期望位置 + 偏航姿态
//   ~/velocity_setpoint  (geometry_msgs::msg::TwistStamped) – 期望速度
//   ~/yaw_setpoint       (std_msgs::msg::Float32)           – 期望偏航角 [rad]
//   /position_cmd        (quadrotor_msgs::msg::PositionCommand) – 完整指令(保持兼容)

// ── 全局句柄（避免每次回调创建时钟）──
rclcpp::Node::SharedPtr g_node_;
rclcpp::Clock::SharedPtr g_clock_;

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_setpoint_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_setpoint_pub_;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_setpoint_pub_;

quadrotor_msgs::msg::PositionCommand cmd;
geometry_msgs::msg::PoseStamped pos_setpoint_;
geometry_msgs::msg::TwistStamped vel_setpoint_;
std_msgs::msg::Float32 yaw_setpoint_;
double pos_gain_[3] = {5.0, 5.0, 5.0};
double vel_gain_[3] = {2.0, 2.0, 2.0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// odometry tracking — 确保控制始终锚定在无人机真实位置
Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
bool have_odom_ = false;
double max_deviation_ = 0.5;  // 允许的最大偏离 [m]

// 静止时间冻结：飞机没动时 t_cur 不往前走
Eigen::Vector3d traj_start_pos_ = Eigen::Vector3d::Zero();  // 收到轨迹时的 odom 位置

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;
  have_odom_ = true;
}

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;
  traj_start_pos_ = odom_pos_;  // 记录轨迹起点（用于静止冻结）

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  // ── 统一时间源：使用节点时钟（与 planner 同步）──
  rclcpp::Time time_now = g_clock_->now();
  double t_cur = (time_now - start_time_).seconds();

  // ── 静止冻结：飞机没动时 t_cur 不往前走，输出恒定速度 ──
  if (have_odom_)
  {
    double moved = (odom_pos_ - traj_start_pos_).norm();
    if (moved < 0.1)
    {
      // 把 t_cur 固定在 0.2s 处（跳过初始瞬态，曲线已稳定）
      t_cur = std::min(t_cur, 0.2);
    }
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = time_now;
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  // ── 偏离检测：平滑收敛，避免位置突变 ──
  static Eigen::Vector3d last_target_pos_ = Eigen::Vector3d::Zero();
  if (have_odom_ && receive_traj_)
  {
    double dev = (odom_pos_ - pos).norm();
    if (dev > max_deviation_)
    {
      // 平滑收敛到当前里程计位置（max 0.03m/tick = 3m/s 修正速率）
      const double max_correction = 0.03;
      Eigen::Vector3d correction = odom_pos_ - pos;
      double corr_norm = correction.norm();
      if (corr_norm > max_correction)
        correction = correction / corr_norm * max_correction;
      pos = pos + correction;   // 平滑过渡，不突变
      vel.setZero();
      acc.setZero();
      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0.0;

      RCLCPP_WARN_THROTTLE(g_node_->get_logger(), *g_clock_, 1000,
          "[Traj server] deviation %.2fm > %.2fm — holding & converging, t_cur=%.2f",
          dev, max_deviation_, t_cur);
    }
    last_target_pos_ = pos;
  }

  // ── 填充自定义消息 (保持兼容) ──
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "odom";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);

  // ── 发布标准化消息 (供任意飞控桥接使用) ──

  // 1. 位置设定点: PoseStamped (位置 + 偏航四元数)
  pos_setpoint_.header.stamp = time_now;
  pos_setpoint_.header.frame_id = "odom";
  pos_setpoint_.pose.position.x = pos(0);
  pos_setpoint_.pose.position.y = pos(1);
  pos_setpoint_.pose.position.z = pos(2);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_yawdot.first);
  pos_setpoint_.pose.orientation.w = q.w();
  pos_setpoint_.pose.orientation.x = q.x();
  pos_setpoint_.pose.orientation.y = q.y();
  pos_setpoint_.pose.orientation.z = q.z();
  pos_setpoint_pub_->publish(pos_setpoint_);

  // 2. 速度设定点: TwistStamped (线速度 + 偏航角速率)
  vel_setpoint_.header.stamp = time_now;
  vel_setpoint_.header.frame_id = "odom";
  vel_setpoint_.twist.linear.x = vel(0);
  vel_setpoint_.twist.linear.y = vel(1);
  vel_setpoint_.twist.linear.z = vel(2);
  vel_setpoint_.twist.angular.z = yaw_yawdot.second;
  vel_setpoint_pub_->publish(vel_setpoint_);

  // 3. 偏航设定点: Float32 [rad]
  yaw_setpoint_.data = yaw_yawdot.first;
  yaw_setpoint_pub_->publish(yaw_setpoint_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");
  g_node_ = node;
  g_clock_ = std::make_shared<rclcpp::Clock>(node->get_clock()->get_clock_type());

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom_world",
      10,
      odomCallback);

  node->declare_parameter("traj_server/max_deviation", 1.0);
  node->get_parameter("traj_server/max_deviation", max_deviation_);

  node->declare_parameter("traj_server/pos_gain_x", 5.0);
  node->declare_parameter("traj_server/pos_gain_y", 5.0);
  node->declare_parameter("traj_server/pos_gain_z", 5.0);
  node->declare_parameter("traj_server/vel_gain_x", 2.0);
  node->declare_parameter("traj_server/vel_gain_y", 2.0);
  node->declare_parameter("traj_server/vel_gain_z", 2.0);

  node->get_parameter("traj_server/pos_gain_x", pos_gain_[0]);
  node->get_parameter("traj_server/pos_gain_y", pos_gain_[1]);
  node->get_parameter("traj_server/pos_gain_z", pos_gain_[2]);
  node->get_parameter("traj_server/vel_gain_x", vel_gain_[0]);
  node->get_parameter("traj_server/vel_gain_y", vel_gain_[1]);
  node->get_parameter("traj_server/vel_gain_z", vel_gain_[2]);

  // ── 自定义消息输出 (保持兼容) ──
  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  // ── 标准化消息输出 (供飞控桥接使用) ──
  pos_setpoint_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/position_setpoint", 10);
  vel_setpoint_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/velocity_setpoint", 10);
  yaw_setpoint_pub_ = node->create_publisher<std_msgs::msg::Float32>(
      "~/yaw_setpoint", 10);

  double publish_rate = 20.0;
  node->declare_parameter("traj_server/publish_rate", 20.0);
  node->get_parameter("traj_server/publish_rate", publish_rate);
  int interval_ms = static_cast<int>(1000.0 / publish_rate);
  RCLCPP_INFO(node->get_logger(), "  Publish rate: %.1f Hz (%d ms)", publish_rate, interval_ms);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain_[0];
  cmd.kx[1] = pos_gain_[1];
  cmd.kx[2] = pos_gain_[2];

  cmd.kv[0] = vel_gain_[0];
  cmd.kv[1] = vel_gain_[1];
  cmd.kv[2] = vel_gain_[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");
  RCLCPP_INFO(node->get_logger(), "  Output interfaces:");
  RCLCPP_INFO(node->get_logger(), "    ~/position_setpoint  (geometry_msgs/PoseStamped)");
  RCLCPP_INFO(node->get_logger(), "    ~/velocity_setpoint  (geometry_msgs/TwistStamped)");
  RCLCPP_INFO(node->get_logger(), "    ~/yaw_setpoint       (std_msgs/Float32)");
  RCLCPP_INFO(node->get_logger(), "    /position_cmd        (quadrotor_msgs/PositionCommand)");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
