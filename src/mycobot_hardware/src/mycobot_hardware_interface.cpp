#include "mycobot_hardware/mycobot_hardware_interface.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mycobot_hardware
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MyCobotHardwareInterface");

namespace
{
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr std::size_t ARM_JOINTS = 6;

// Names of the 6 arm joints in command order. Matched as suffixes so any URDF
// prefix (e.g. "mycobot_") is tolerated.
const std::array<std::string, ARM_JOINTS> ARM_SUFFIXES = {
  "link1_to_link2", "link2_to_link3", "link3_to_link4",
  "link4_to_link5", "link5_to_link6", "link6_to_link6_flange"};

bool endsWith(const std::string & s, const std::string & suffix)
{
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

// ── lifecycle ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_ip_ = info_.hardware_parameters.count("robot_ip")
    ? info_.hardware_parameters.at("robot_ip") : "192.168.0.101";
  robot_port_ = info_.hardware_parameters.count("robot_port")
    ? std::stoi(info_.hardware_parameters.at("robot_port")) : 9000;
  joint_speed_ = info_.hardware_parameters.count("joint_speed")
    ? std::stoi(info_.hardware_parameters.at("joint_speed")) : 30;

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Locate the 6 arm joints by name suffix; record their indices in command order.
  arm_indices_.assign(ARM_JOINTS, info_.joints.size());
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (std::size_t a = 0; a < ARM_JOINTS; ++a) {
      if (endsWith(info_.joints[i].name, ARM_SUFFIXES[a])) {
        arm_indices_[a] = i;
      }
    }
  }
  for (std::size_t a = 0; a < ARM_JOINTS; ++a) {
    if (arm_indices_[a] >= info_.joints.size()) {
      RCLCPP_ERROR(LOGGER, "Could not find arm joint matching '*%s' in URDF",
        ARM_SUFFIXES[a].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  arm_cmd_deg_.assign(ARM_JOINTS, 0.0);
  arm_pos_deg_.assign(ARM_JOINTS, 0.0);

  RCLCPP_INFO(LOGGER, "Initialized — robot %s:%d, speed %d, %zu joints (%zu arm mapped)",
    robot_ip_.c_str(), robot_port_, joint_speed_, info_.joints.size(), ARM_JOINTS);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!connectSocket()) {
    RCLCPP_ERROR(LOGGER, "Could not connect to bridge at %s:%d (is mycobot_tcp_bridge.py running?)",
      robot_ip_.c_str(), robot_port_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string reply;
  if (!transact("PING", reply) || reply != "PONG") {
    RCLCPP_ERROR(LOGGER, "Bridge did not answer PING (got '%s')", reply.c_str());
    closeSocket();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "Connected to bridge at %s:%d", robot_ip_.c_str(), robot_port_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Seed commands from the robot's current pose so it does NOT jump on start.
  std::string reply;
  if (transact("GET_ANGLES", reply) && reply != "ERR") {
    std::stringstream ss(reply);
    std::string tok;
    std::size_t a = 0;
    while (std::getline(ss, tok, ',') && a < ARM_JOINTS) {
      const double deg = std::stod(tok);
      arm_pos_deg_[a] = deg;
      arm_cmd_deg_[a] = deg;
      hw_positions_[arm_indices_[a]] = deg * DEG2RAD;
      hw_commands_[arm_indices_[a]] = deg * DEG2RAD;
      ++a;
    }
    have_pos_ = (a == ARM_JOINTS);
  }
  if (!have_pos_) {
    RCLCPP_ERROR(LOGGER, "Failed to read initial arm angles on activate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Mirror non-arm joints (gripper_controller + mimics): command -> position.
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_commands_[i] = hw_positions_[i];
  }

  comm_running_ = true;
  comm_thread_ = std::thread(&MyCobotHardwareInterface::commLoop, this);

  RCLCPP_INFO(LOGGER, "Activated — comm thread started");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comm_running_ = false;
  if (comm_thread_.joinable()) {
    comm_thread_.join();
  }
  closeSocket();
  RCLCPP_INFO(LOGGER, "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── interface export (respect what the URDF declares) ────────────────────────

std::vector<hardware_interface::StateInterface>
MyCobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      }
      // velocity state interfaces (gripper mimics) are declared in the URDF but
      // not driven here; ros2_control tolerates them being unexported.
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyCobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
      }
    }
  }
  return interfaces;
}

// ── read / write (non-blocking; just touch caches) ──────────────────────────

hardware_interface::return_type MyCobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  {
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (have_pos_) {
      for (std::size_t a = 0; a < ARM_JOINTS; ++a) {
        hw_positions_[arm_indices_[a]] = arm_pos_deg_[a] * DEG2RAD;
      }
    }
  }
  // Non-arm joints (gripper_controller + mimics): mirror command -> position.
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    bool is_arm = false;
    for (std::size_t a = 0; a < ARM_JOINTS; ++a) {
      if (arm_indices_[a] == i) { is_arm = true; break; }
    }
    if (!is_arm) {
      hw_positions_[i] = hw_commands_[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  for (std::size_t a = 0; a < ARM_JOINTS; ++a) {
    arm_cmd_deg_[a] = hw_commands_[arm_indices_[a]] * RAD2DEG;
  }
  return hardware_interface::return_type::OK;
}

// ── background comm thread ───────────────────────────────────────────────────

void MyCobotHardwareInterface::commLoop()
{
  using namespace std::chrono_literals;
  while (comm_running_) {
    std::vector<double> cmd_deg(ARM_JOINTS);
    {
      std::lock_guard<std::mutex> lock(comm_mutex_);
      cmd_deg = arm_cmd_deg_;
    }

    std::ostringstream req;
    req << "SET_ANGLES";
    for (double d : cmd_deg) {
      req << ' ' << d;
    }
    req << ' ' << joint_speed_;

    std::string reply;
    if (!transact(req.str(), reply)) {
      RCLCPP_WARN(LOGGER, "SET_ANGLES transaction failed; robot comms lost?");
      std::this_thread::sleep_for(50ms);
      continue;
    }

    if (transact("GET_ANGLES", reply) && reply != "ERR") {
      std::vector<double> pos;
      std::stringstream ss(reply);
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        try { pos.push_back(std::stod(tok)); } catch (...) { pos.clear(); break; }
      }
      if (pos.size() == ARM_JOINTS) {
        std::lock_guard<std::mutex> lock(comm_mutex_);
        arm_pos_deg_ = pos;
        have_pos_ = true;
      }
    }

    std::this_thread::sleep_for(20ms);  // ~50 Hz cap on robot round-trips
  }
}

// ── TCP helpers ──────────────────────────────────────────────────────────────

bool MyCobotHardwareInterface::connectSocket()
{
  sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd_ < 0) {
    return false;
  }

  int one = 1;
  ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(robot_port_));
  if (::inet_pton(AF_INET, robot_ip_.c_str(), &addr.sin_addr) <= 0) {
    closeSocket();
    return false;
  }

  if (::connect(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    closeSocket();
    return false;
  }
  return true;
}

void MyCobotHardwareInterface::closeSocket()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

bool MyCobotHardwareInterface::transact(const std::string & request, std::string & reply)
{
  if (sock_fd_ < 0) {
    return false;
  }
  const std::string line = request + "\n";
  std::size_t sent = 0;
  while (sent < line.size()) {
    const ssize_t n = ::send(sock_fd_, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }

  // Read until newline.
  reply.clear();
  char c;
  while (true) {
    const ssize_t n = ::recv(sock_fd_, &c, 1, 0);
    if (n <= 0) {
      return false;
    }
    if (c == '\n') {
      break;
    }
    reply.push_back(c);
  }
  return true;
}

}  // namespace mycobot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mycobot_hardware::MyCobotHardwareInterface,
  hardware_interface::SystemInterface)
