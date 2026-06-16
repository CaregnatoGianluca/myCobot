#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mycobot_hardware
{

// Drives the real myCobot 280 over TCP. The robot's Raspberry Pi runs
// mycobot_tcp_bridge.py (see scripts/), which speaks a simple newline-delimited
// ASCII protocol on top of the working pymycobot direct-serial path.
//
// A background comm thread does the (slow) network/serial round-trips so the
// 100 Hz ros2_control loop never blocks: read()/write() only touch in-memory
// caches guarded by a mutex.
class MyCobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyCobotHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── TCP helpers (POSIX sockets) ────────────────────────────────────────────
  bool connectSocket();
  void closeSocket();
  // Send one command line (no newline needed) and read one reply line.
  // Returns false on any socket error; reply has the trailing newline stripped.
  bool transact(const std::string & request, std::string & reply);

  // Background loop: push latest arm commands, pull latest arm positions.
  void commLoop();

  // ── connection params (from URDF <hardware><param> tags) ────────────────────
  std::string robot_ip_;
  int robot_port_{9000};
  int joint_speed_{30};

  int sock_fd_{-1};

  // ── joint bookkeeping ───────────────────────────────────────────────────────
  // ros2_control storage (radians), one slot per URDF joint. Indices of the 6
  // arm joints (the only ones mapped to the real robot) are in arm_indices_,
  // ordered link1_to_link2 .. link6_to_link6_flange.
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::vector<std::size_t> arm_indices_;

  // Shared with the comm thread (degrees).
  std::mutex comm_mutex_;
  std::vector<double> arm_cmd_deg_;   // latest commanded arm angles to send
  std::vector<double> arm_pos_deg_;   // latest arm angles read back
  bool have_pos_{false};              // arm_pos_deg_ has valid data yet

  std::thread comm_thread_;
  std::atomic<bool> comm_running_{false};
};

}  // namespace mycobot_hardware
