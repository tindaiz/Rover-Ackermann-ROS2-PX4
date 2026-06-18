#include "ackermann_nav2_behaviors/ackermann_turnaround.hpp"

#include <memory>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ackermann_nav2_behaviors
{

AckermannTurnAround::AckermannTurnAround()
: TimedBehavior<SpinAction>()
{
}

AckermannTurnAround::~AckermannTurnAround() = default;

void AckermannTurnAround::onConfigure()
{
  auto node = this->node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".reverse_speed", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".forward_speed", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".angular_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(2.0));

  node->get_parameter(this->behavior_name_ + ".reverse_speed", reverse_speed_);
  node->get_parameter(this->behavior_name_ + ".forward_speed", forward_speed_);
  node->get_parameter(this->behavior_name_ + ".angular_vel", angular_vel_);
  node->get_parameter(this->behavior_name_ + ".simulate_ahead_time", simulate_ahead_time_);
}

AckermannTurnAround::Status AckermannTurnAround::onRun(
  const std::shared_ptr<const SpinAction::Goal> command)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);
  relative_yaw_ = 0.0;
  cmd_yaw_ = command->target_yaw;

  feedback_ = std::make_shared<SpinAction::Feedback>();
  
  direction_ = REVERSE;

  command_time_allowance_ = command->time_allowance;
  if (command_time_allowance_.seconds() < 4.0 && command_time_allowance_.seconds() > 0.0) {
    command_time_allowance_ = rclcpp::Duration::from_seconds(10.0);
  } else if (command_time_allowance_.seconds() == 0.0) {
    command_time_allowance_ = rclcpp::Duration::from_seconds(10.0);
  }

  end_time_ = this->clock_->now() + command_time_allowance_;

  RCLCPP_INFO(this->logger_, "Starting Ackermann turnaround recovery (Target Yaw: %.2f)", cmd_yaw_);

  return Status::SUCCEEDED;
}

AckermannTurnAround::Status AckermannTurnAround::onCycleUpdate()
{
  auto current_time = this->clock_->now();
  rclcpp::Duration time_remaining = end_time_ - current_time;

  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Turnaround timeout");
    return Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double delta_yaw = current_yaw - prev_yaw_;
  if (std::abs(delta_yaw) > M_PI) {
    delta_yaw = copysign(2 * M_PI - std::abs(delta_yaw), prev_yaw_);
  }

  relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;

  feedback_->angular_distance_traveled = static_cast<float>(relative_yaw_);
  this->action_server_->publish_feedback(feedback_);

  double remaining_yaw = std::abs(cmd_yaw_) - std::abs(relative_yaw_);
  if (remaining_yaw < 0.05) { // 0.05 rad tolerance
    this->stopRobot();
    RCLCPP_INFO(this->logger_, "Ackermann turnaround completed successfully");
    return Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = (direction_ == FORWARD) ? forward_speed_ : -reverse_speed_;
  cmd_vel->angular.z = copysign(angular_vel_, cmd_yaw_);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(cmd_vel.get(), pose2d)) {
    // Toggle direction
    direction_ = (direction_ == FORWARD) ? REVERSE : FORWARD;
    cmd_vel->linear.x = (direction_ == FORWARD) ? forward_speed_ : -reverse_speed_;
    
    // Check if the new direction is also blocked
    geometry_msgs::msg::Pose2D new_pose2d = pose2d;
    if (!isCollisionFree(cmd_vel.get(), new_pose2d)) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Blocked in both directions - Exiting Turnaround");
      return Status::FAILED;
    }
  }

  this->vel_pub_->publish(std::move(cmd_vel));

  return Status::RUNNING;
}

bool AckermannTurnAround::isCollisionFree(
  geometry_msgs::msg::Twist * cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  int cycle_count = 0;
  const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
  bool fetch_data = true;
  double dt = 1.0 / this->cycle_frequency_;

  while (cycle_count < max_cycle_count) {
    pose2d.x += cmd_vel->linear.x * cos(pose2d.theta) * dt;
    pose2d.y += cmd_vel->linear.x * sin(pose2d.theta) * dt;
    pose2d.theta += cmd_vel->angular.z * dt;

    cycle_count++;

    if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace ackermann_nav2_behaviors

PLUGINLIB_EXPORT_CLASS(
  ackermann_nav2_behaviors::AckermannTurnAround,
  nav2_core::Behavior)