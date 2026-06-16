#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "mycobot_interfaces/action/move_to_joints.hpp"
#include "mycobot_interfaces/action/move_to_named.hpp"
#include "mycobot_interfaces/action/move_to_pose.hpp"

using MoveToPose   = mycobot_interfaces::action::MoveToPose;
using MoveToJoints = mycobot_interfaces::action::MoveToJoints;
using MoveToNamed  = mycobot_interfaces::action::MoveToNamed;
using namespace std::chrono_literals;

// Sends a goal, spins the node until the result arrives, returns success flag.
template <typename ActionT>
bool sendGoal(
  rclcpp::Node::SharedPtr node,
  typename rclcpp_action::Client<ActionT>::SharedPtr client,
  const typename ActionT::Goal & goal,
  const std::string & description)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Sending goal: %s", description.c_str());

  auto options = typename rclcpp_action::Client<ActionT>::SendGoalOptions{};
  options.feedback_callback =
    [&logger](auto, const auto & fb) {
      RCLCPP_INFO(logger, "  feedback: %s", fb->state.c_str());
    };

  auto future_handle = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, future_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Goal was rejected or send failed");
    return false;
  }

  auto goal_handle = future_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger, "Goal rejected by server");
    return false;
  }

  auto future_result = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Failed to get result");
    return false;
  }

  const auto & result = future_result.get();
  const bool ok = result.result->success;
  RCLCPP_INFO(logger, "  result: %s — %s",
    ok ? "SUCCESS" : "FAILED", result.result->message.c_str());
  return ok;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_demo");

  auto named_client  = rclcpp_action::create_client<MoveToNamed>(node, "move_to_named");
  auto joints_client = rclcpp_action::create_client<MoveToJoints>(node, "move_to_joints");
  auto pose_client   = rclcpp_action::create_client<MoveToPose>(node, "move_to_pose");

  RCLCPP_INFO(node->get_logger(), "Waiting for motion_server action servers...");
  named_client->wait_for_action_server(30s);
  joints_client->wait_for_action_server(30s);
  pose_client->wait_for_action_server(30s);
  RCLCPP_INFO(node->get_logger(), "All action servers ready.");

  // ── Demo 1: named target — move arm to "ready" ───────────────────────────
  {
    MoveToNamed::Goal g;
    g.group_name     = "arm";
    g.target_name    = "ready";
    g.velocity_scale = 0.1;
    sendGoal<MoveToNamed>(node, named_client, g, "arm → ready");
    rclcpp::sleep_for(1s);
  }

  // ── Demo 2: joint angles — all zeros (home) ──────────────────────────────
  {
    MoveToJoints::Goal g;
    g.joint_angles       = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    g.velocity_scale     = 0.1;
    g.acceleration_scale = 0.1;
    sendGoal<MoveToJoints>(node, joints_client, g, "arm → home via joints");
    rclcpp::sleep_for(1s);
  }

  // ── Demo 3: Cartesian pose (above table centre, reachable) ───────────────
  {
    MoveToPose::Goal g;
    g.target_pose.header.frame_id = "world";
    // x=0.2 m forward, z=0.3 m above base — well within the 280 mm reach
    g.target_pose.pose.position.x    =  0.20;
    g.target_pose.pose.position.y    =  0.00;
    g.target_pose.pose.position.z    =  0.30;
    // EEF pointing straight down
    g.target_pose.pose.orientation.x =  0.0;
    g.target_pose.pose.orientation.y =  0.707;
    g.target_pose.pose.orientation.z =  0.0;
    g.target_pose.pose.orientation.w =  0.707;
    g.velocity_scale     = 0.1;
    g.acceleration_scale = 0.1;
    g.planning_pipeline  = "ompl";
    sendGoal<MoveToPose>(node, pose_client, g, "arm → Cartesian pose");
    rclcpp::sleep_for(1s);
  }

  // ── Demo 4: gripper open / close ─────────────────────────────────────────
  {
    MoveToNamed::Goal g;
    g.group_name     = "gripper";
    g.target_name    = "open";
    g.velocity_scale = 0.2;
    sendGoal<MoveToNamed>(node, named_client, g, "gripper → open");

    g.target_name = "closed";
    sendGoal<MoveToNamed>(node, named_client, g, "gripper → closed");
  }

  // ── Return home ──────────────────────────────────────────────────────────
  {
    MoveToNamed::Goal g;
    g.group_name     = "arm";
    g.target_name    = "home";
    g.velocity_scale = 0.1;
    sendGoal<MoveToNamed>(node, named_client, g, "arm → home");
  }

  RCLCPP_INFO(node->get_logger(), "Demo complete.");
  rclcpp::shutdown();
  return 0;
}
