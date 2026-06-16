#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mycobot_motion
{

struct MotionResult
{
  bool success{false};
  std::string message;
  geometry_msgs::msg::PoseStamped final_pose;
  std::vector<double> final_joints;
};

/**
 * Thin wrapper around MoveGroupInterface for the myCobot 280.
 *
 * Constructed with a live rclcpp::Node (must already be spinning in a
 * MultiThreadedExecutor so that TF and joint-state callbacks are processed
 * while plan()/execute() block).
 *
 * All move*() methods are blocking and thread-safe as long as each call
 * completes before the next one starts (MoveGroupInterface is not re-entrant).
 */
class MotionManager
{
public:
  explicit MotionManager(rclcpp::Node::SharedPtr node);

  /**
   * Move the arm EEF to a Cartesian pose.
   * @param target   Desired pose (frame_id should be "world" or "base_link").
   * @param vel      Velocity scaling factor [0, 1].
   * @param accel    Acceleration scaling factor [0, 1].
   * @param pipeline Planning pipeline: "ompl", "pilz_industrial_motion_planner", "stomp".
   */
  MotionResult moveToPose(
    const geometry_msgs::msg::PoseStamped & target,
    double vel   = 0.1,
    double accel = 0.1,
    const std::string & pipeline = "ompl");

  /**
   * Move the arm to an explicit joint configuration.
   * @param angles  6 joint values in radians, in arm-group order:
   *                [link1_to_link2, link2_to_link3, ..., link6_to_link6_flange].
   */
  MotionResult moveToJoints(
    const std::vector<double> & angles,
    double vel   = 0.1,
    double accel = 0.1);

  /**
   * Move a planning group to a named state defined in the SRDF.
   * @param group   "arm", "gripper", or "arm_with_gripper".
   * @param target  Named state: arm → "home","ready"; gripper → "open","half_closed","closed".
   */
  MotionResult moveToNamed(
    const std::string & group,
    const std::string & target,
    double vel = 0.1);

  geometry_msgs::msg::PoseStamped getCurrentPose() const;
  std::vector<double> getCurrentJointValues() const;

private:
  rclcpp::Node::SharedPtr node_;

  // Separate MoveGroupInterface per group — created once in the constructor.
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

  // Parameters (loaded from node parameters with fallback defaults).
  std::string arm_group_;
  std::string gripper_group_;
  std::string eef_link_;
  std::string default_pipeline_;
  double planning_time_;
  int num_planning_attempts_;

  // Returns the MoveGroupInterface for the requested group name.
  // Throws std::invalid_argument if the group is unknown.
  moveit::planning_interface::MoveGroupInterface & groupFor(const std::string & group_name);
};

}  // namespace mycobot_motion
