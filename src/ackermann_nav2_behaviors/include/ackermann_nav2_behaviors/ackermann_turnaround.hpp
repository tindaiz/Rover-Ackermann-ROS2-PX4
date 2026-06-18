#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace ackermann_nav2_behaviors
{

class AckermannTurnAround
  : public nav2_behaviors::TimedBehavior<nav2_msgs::action::Spin>
{
public:
  using SpinAction = nav2_msgs::action::Spin;
  using Status = nav2_behaviors::Status;

  AckermannTurnAround();
  ~AckermannTurnAround();

  void onConfigure() override;

  Status onRun(
    const std::shared_ptr<const SpinAction::Goal> command) override;

  Status onCycleUpdate() override;

protected:
  bool isCollisionFree(
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  double reverse_speed_;
  double forward_speed_;
  double angular_vel_;
  double simulate_ahead_time_;

  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;

  std::shared_ptr<SpinAction::Feedback> feedback_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_{0, 0, RCL_ROS_TIME};

  enum Direction
  {
    REVERSE,
    FORWARD
  };

  Direction direction_;
};

}  // namespace ackermann_nav2_behaviors