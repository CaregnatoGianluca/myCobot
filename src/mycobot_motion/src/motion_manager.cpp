#include "mycobot_motion/motion_manager.hpp"

#include <stdexcept>

namespace mycobot_motion
{

MotionManager::MotionManager(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Load tunable parameters declared by the caller node.
  // motion_server_node declares these before constructing MotionManager.
  node_->get_parameter_or("arm_group",               arm_group_,             std::string("arm"));
  node_->get_parameter_or("gripper_group",            gripper_group_,         std::string("gripper"));
  node_->get_parameter_or("eef_link",                 eef_link_,              std::string("link6_flange"));
  node_->get_parameter_or("default_planning_pipeline", default_pipeline_,     std::string("ompl"));
  node_->get_parameter_or("planning_time",            planning_time_,         5.0);
  node_->get_parameter_or("num_planning_attempts",    num_planning_attempts_, 10);

  // MoveGroupInterface connects to move_group over ROS 2 topics.
  // The node must already be added to a spinning executor at this point.
  arm_     = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_);
  gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, gripper_group_);

  arm_->setEndEffectorLink(eef_link_);
  arm_->setPlanningTime(planning_time_);
  arm_->setNumPlanningAttempts(num_planning_attempts_);

  gripper_->setPlanningTime(planning_time_);

  RCLCPP_INFO(node_->get_logger(),
    "MotionManager ready — arm group: '%s', gripper group: '%s', EEF: '%s'",
    arm_group_.c_str(), gripper_group_.c_str(), eef_link_.c_str());
}

// ── helpers ──────────────────────────────────────────────────────────────────

moveit::planning_interface::MoveGroupInterface &
MotionManager::groupFor(const std::string & group_name)
{
  if (group_name == arm_group_ || group_name == "arm_with_gripper") {
    return *arm_;
  }
  if (group_name == gripper_group_) {
    return *gripper_;
  }
  throw std::invalid_argument("Unknown planning group: '" + group_name + "'");
}

static void applyScaling(
  moveit::planning_interface::MoveGroupInterface & g,
  double vel, double accel)
{
  g.setMaxVelocityScalingFactor(std::clamp(vel,   0.01, 1.0));
  g.setMaxAccelerationScalingFactor(std::clamp(accel, 0.01, 1.0));
}

// ── public API ───────────────────────────────────────────────────────────────

MotionResult MotionManager::moveToPose(
  const geometry_msgs::msg::PoseStamped & target,
  double vel, double accel,
  const std::string & pipeline)
{
  MotionResult result;

  const std::string pipe = pipeline.empty() ? default_pipeline_ : pipeline;
  arm_->setPlanningPipelineId(pipe);
  applyScaling(*arm_, vel, accel);
  arm_->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    result.message = "Planning failed (pipeline: " + pipe + ")";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    arm_->clearPoseTargets();
    return result;
  }

  const bool executed = (arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  arm_->clearPoseTargets();

  if (!executed) {
    result.message = "Execution failed";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  result.success    = true;
  result.message    = "ok";
  result.final_pose = arm_->getCurrentPose(eef_link_);
  result.final_joints = arm_->getCurrentJointValues();
  return result;
}

MotionResult MotionManager::moveToJoints(
  const std::vector<double> & angles,
  double vel, double accel)
{
  MotionResult result;

  applyScaling(*arm_, vel, accel);

  const bool set_ok = arm_->setJointValueTarget(angles);
  if (!set_ok) {
    result.message = "setJointValueTarget failed — check joint count and limits";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    result.message = "Planning failed";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  const bool executed = (arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    result.message = "Execution failed";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  result.success      = true;
  result.message      = "ok";
  result.final_pose   = arm_->getCurrentPose(eef_link_);
  result.final_joints = arm_->getCurrentJointValues();
  return result;
}

MotionResult MotionManager::moveToNamed(
  const std::string & group,
  const std::string & target,
  double vel)
{
  MotionResult result;

  moveit::planning_interface::MoveGroupInterface * g = nullptr;
  try {
    g = &groupFor(group);
  } catch (const std::invalid_argument & e) {
    result.message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  applyScaling(*g, vel, vel);  // use same factor for both vel and accel

  const bool set_ok = g->setNamedTarget(target);
  if (!set_ok) {
    result.message = "Unknown named target '" + target + "' for group '" + group + "'";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = (g->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    result.message = "Planning failed for named target '" + target + "'";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  const bool executed = (g->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!executed) {
    result.message = "Execution failed";
    RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  result.success      = true;
  result.message      = "ok";
  result.final_joints = g->getCurrentJointValues();
  return result;
}

geometry_msgs::msg::PoseStamped MotionManager::getCurrentPose() const
{
  return arm_->getCurrentPose(eef_link_);
}

std::vector<double> MotionManager::getCurrentJointValues() const
{
  return arm_->getCurrentJointValues();
}

}  // namespace mycobot_motion
