#include "mycobot_motion/motion_manager.hpp"

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "mycobot_interfaces/action/move_to_joints.hpp"
#include "mycobot_interfaces/action/move_to_named.hpp"
#include "mycobot_interfaces/action/move_to_pose.hpp"

using MoveToPose   = mycobot_interfaces::action::MoveToPose;
using MoveToJoints = mycobot_interfaces::action::MoveToJoints;
using MoveToNamed  = mycobot_interfaces::action::MoveToNamed;

class MotionServerNode : public rclcpp::Node
{
public:
  MotionServerNode()
  : rclcpp::Node("motion_server")
  {
    // Declare parameters before MotionManager reads them.
    declare_parameter("arm_group",                "arm");
    declare_parameter("gripper_group",            "gripper");
    declare_parameter("eef_link",                 "link6_flange");
    declare_parameter("default_velocity_scale",   0.1);
    declare_parameter("default_acceleration_scale", 0.1);
    declare_parameter("default_planning_pipeline", "ompl");
    declare_parameter("planning_time",            5.0);
    declare_parameter("num_planning_attempts",    10);
  }

  // Called after shared_from_this() is available (i.e., after make_shared).
  void initialize()
  {
    motion_manager_ = std::make_unique<mycobot_motion::MotionManager>(shared_from_this());

    pose_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_to_pose",
      [this](auto, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](auto goal_handle) {
        std::thread([this, goal_handle]() { executePose(goal_handle); }).detach();
      });

    joints_server_ = rclcpp_action::create_server<MoveToJoints>(
      this, "move_to_joints",
      [this](auto, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](auto goal_handle) {
        std::thread([this, goal_handle]() { executeJoints(goal_handle); }).detach();
      });

    named_server_ = rclcpp_action::create_server<MoveToNamed>(
      this, "move_to_named",
      [this](auto, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](auto goal_handle) {
        std::thread([this, goal_handle]() { executeNamed(goal_handle); }).detach();
      });

    RCLCPP_INFO(get_logger(), "MotionServerNode ready — actions: /move_to_pose, /move_to_joints, /move_to_named");
  }

private:
  std::unique_ptr<mycobot_motion::MotionManager> motion_manager_;
  rclcpp_action::Server<MoveToPose>::SharedPtr   pose_server_;
  rclcpp_action::Server<MoveToJoints>::SharedPtr joints_server_;
  rclcpp_action::Server<MoveToNamed>::SharedPtr  named_server_;

  // ── MoveToPose ─────────────────────────────────────────────────────────────
  void executePose(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle)
  {
    const auto & goal = goal_handle->get_goal();

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->state = "planning";
    goal_handle->publish_feedback(feedback);

    const double vel   = goal->velocity_scale   > 0.0 ? goal->velocity_scale   : 0.1;
    const double accel = goal->acceleration_scale > 0.0 ? goal->acceleration_scale : 0.1;

    auto mr = motion_manager_->moveToPose(
      goal->target_pose, vel, accel, goal->planning_pipeline);

    feedback->state = "executing";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<MoveToPose::Result>();
    result->success       = mr.success;
    result->message       = mr.message;
    result->achieved_pose = mr.final_pose;

    if (mr.success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ── MoveToJoints ───────────────────────────────────────────────────────────
  void executeJoints(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>> goal_handle)
  {
    const auto & goal = goal_handle->get_goal();

    auto feedback = std::make_shared<MoveToJoints::Feedback>();
    feedback->state = "planning";
    goal_handle->publish_feedback(feedback);

    const double vel   = goal->velocity_scale   > 0.0 ? goal->velocity_scale   : 0.1;
    const double accel = goal->acceleration_scale > 0.0 ? goal->acceleration_scale : 0.1;

    auto mr = motion_manager_->moveToJoints(
      std::vector<double>(goal->joint_angles.begin(), goal->joint_angles.end()),
      vel, accel);

    feedback->state = "executing";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<MoveToJoints::Result>();
    result->success         = mr.success;
    result->message         = mr.message;
    result->achieved_joints = mr.final_joints;

    if (mr.success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ── MoveToNamed ────────────────────────────────────────────────────────────
  void executeNamed(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> goal_handle)
  {
    const auto & goal = goal_handle->get_goal();

    auto feedback = std::make_shared<MoveToNamed::Feedback>();
    feedback->state = "planning";
    goal_handle->publish_feedback(feedback);

    const double vel = goal->velocity_scale > 0.0 ? goal->velocity_scale : 0.1;

    auto mr = motion_manager_->moveToNamed(goal->group_name, goal->target_name, vel);

    feedback->state = "executing";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<MoveToNamed::Result>();
    result->success = mr.success;
    result->message = mr.message;

    if (mr.success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MultiThreadedExecutor is required: MoveGroupInterface needs its own
  // callback thread for TF/joint-state subscriptions while plan()/execute()
  // block inside the action handler threads.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MotionServerNode>();
  node->initialize();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
