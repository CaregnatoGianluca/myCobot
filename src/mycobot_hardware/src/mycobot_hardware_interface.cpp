#include "mycobot_hardware/mycobot_hardware_interface.hpp"

#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mycobot_hardware
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MyCobotHardwareInterface");

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port") : "/dev/ttyAMA0";
  baud_rate_ = info_.hardware_parameters.count("baud_rate")
    ? std::stoi(info_.hardware_parameters.at("baud_rate")) : 1000000;

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(LOGGER, "Initialized — port: %s @ %d baud", serial_port_.c_str(), baud_rate_);
  RCLCPP_WARN(LOGGER, "Real serial comms not yet implemented. Stub mode active.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(Phase 1): open serial port and verify connection with pymycobot
  RCLCPP_INFO(LOGGER, "Configured (stub — no serial connection opened)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_commands_[i] = hw_positions_[i];
  }
  RCLCPP_INFO(LOGGER, "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(Phase 1): send arm to safe home position before closing serial
  RCLCPP_INFO(LOGGER, "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyCobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyCobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type MyCobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(Phase 1): call pymycobot get_angles(), convert degrees → radians
  // Mirror commands as positions until real comms are wired up
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(Phase 1): convert radians → degrees, call pymycobot set_angles()
  return hardware_interface::return_type::OK;
}

}  // namespace mycobot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mycobot_hardware::MyCobotHardwareInterface,
  hardware_interface::SystemInterface)
