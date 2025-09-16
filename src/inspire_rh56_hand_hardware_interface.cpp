// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************

#include "inspire_rh56_hand_ros2_control/inspire_rh56_hand_hardware_interface.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inspire_rh56_hand_ros2_control
{

hardware_interface::CallbackReturn InspireRH56HandHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Initialize parameters
  serial_port_ = info_.hardware_parameters["serial_port"];
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  hand_id_ = 1;  // Default hand ID

  RCLCPP_INFO(
    rclcpp::get_logger("InspireRH56HandHardwareInterface"),
    "Serial port: %s, Baudrate: %d, Hand ID: %d", serial_port_.c_str(), baudrate_, hand_id_);

  // Initialize joint vectors
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);

  // Initialize state tracking
  first_read_completed_ = false;

  // Initialize joint limits from URDF/XACRO definitions
  joint_min_limits_.resize(info_.joints.size());
  joint_max_limits_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // Extract limits from URDF joint info
    const auto & joint = info_.joints[i];
    joint_min_limits_[i] = std::stod(joint.command_interfaces[0].min);
    joint_max_limits_[i] = std::stod(joint.command_interfaces[0].max);

    RCLCPP_INFO(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Joint %s: min=%.3f, max=%.3f",
      joint.name.c_str(), joint_min_limits_[i], joint_max_limits_[i]);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
InspireRH56HandHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
InspireRH56HandHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn InspireRH56HandHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Activating...");

  if (!open_serial_port()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to open serial port %s",
      serial_port_.c_str());
    return CallbackReturn::ERROR;
  }

  // Reset state tracking - let first read() call initialize positions
  first_read_completed_ = false;

  RCLCPP_INFO(rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Successfully activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireRH56HandHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Deactivating...");
  close_serial_port();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type InspireRH56HandHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read current angles from ANGLE_ACT registers
  auto current_angles = read_angle_act_registers();
  if (current_angles.size() != NUM_JOINTS) {
    RCLCPP_WARN(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to read joint positions");
    return hardware_interface::return_type::ERROR;
  }

  // Store previous positions for velocity calculation
  std::vector<double> prev_positions = hw_positions_;

  // Convert hardware values to joint positions
  for (size_t urdf_idx = 0; urdf_idx < NUM_JOINTS; ++urdf_idx) {
    size_t hw_idx = urdf_to_hw_index(urdf_idx);
    hw_positions_[urdf_idx] = hardware_value_to_position(current_angles[hw_idx], urdf_idx);
  }

  // Calculate joint velocities using finite difference
  if (first_read_completed_ && period.seconds() > 0.0) {
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      hw_velocities_[i] = (hw_positions_[i] - prev_positions[i]) / period.seconds();
    }
  } else {
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  }

  // Initialize commands to current positions on first read
  if (!first_read_completed_) {
    std::copy(hw_positions_.begin(), hw_positions_.end(), hw_commands_.begin());
    first_read_completed_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"),
      "First read completed - initialized positions and commands");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InspireRH56HandHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert URDF joint commands to hardware register values
  std::vector<int16_t> target_angles(NUM_JOINTS);
  for (size_t urdf_idx = 0; urdf_idx < NUM_JOINTS; ++urdf_idx) {
    size_t hw_idx = urdf_to_hw_index(urdf_idx);
    target_angles[hw_idx] = position_to_hardware_value(hw_commands_[urdf_idx], urdf_idx);
  }

  // Write to ANGLE_SET registers
  if (!write_angle_set_registers(target_angles)) {
    RCLCPP_WARN(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to write joint commands");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// RS-485 Communication Implementation

bool InspireRH56HandHardwareInterface::open_serial_port()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Error opening %s: %s",
      serial_port_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (::tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Error from tcgetattr: %s",
      strerror(errno));
    return false;
  }

  // Set baud rate
  speed_t speed = B115200;  // Default to 115200
  if (baudrate_ == 115200)
    speed = B115200;
  else if (baudrate_ == 57600)
    speed = B57600;
  else if (baudrate_ == 19200)
    speed = B19200;

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (::tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Error from tcsetattr: %s",
      strerror(errno));
    return false;
  }

  return true;
}

void InspireRH56HandHardwareInterface::close_serial_port()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool InspireRH56HandHardwareInterface::send_read_frame(
  uint16_t address, uint8_t length, std::vector<uint8_t> & response)
{
  // Read frame: EB 90 | ID | 04 | 11 | Addr_L | Addr_H | Len | Chk
  std::vector<uint8_t> frame = {
    0xEB,
    0x90,                                         // Header
    hand_id_,                                     // Hand ID
    0x04,                                         // Fixed length for read command
    0x11,                                         // Read command
    static_cast<uint8_t>(address & 0xFF),         // Address low byte
    static_cast<uint8_t>((address >> 8) & 0xFF),  // Address high byte
    length                                        // Data length to read
  };

  // Calculate checksum
  uint32_t sum = std::accumulate(frame.begin() + 2, frame.end(), 0u);
  uint8_t checksum = static_cast<uint8_t>(sum & 0xFF);
  frame.push_back(checksum);

  // Send frame
  if (::write(serial_fd_, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to send read frame");
    return false;
  }

  // Read response: 90 EB | ID | Len | 0x11 | Addr_L | Addr_H | Data... | Chk
  std::vector<uint8_t> resp_header(7);
  if (::read(serial_fd_, resp_header.data(), 7) != 7) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to read response header");
    return false;
  }

  // Verify response header
  if (
    resp_header[0] != 0x90 || resp_header[1] != 0xEB || resp_header[2] != hand_id_ ||
    resp_header[4] != 0x11) {
    RCLCPP_ERROR(rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Invalid response header");
    return false;
  }

  uint8_t resp_length = resp_header[3] - 3;
  response.resize(resp_length + 1);  // +1 for checksum

  if (::read(serial_fd_, response.data(), resp_length + 1) != resp_length + 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to read response data");
    return false;
  }

  // Verify checksum
  std::vector<uint8_t> check_data(resp_header.begin() + 2, resp_header.end());
  check_data.insert(check_data.end(), response.begin(), response.end() - 1);
  uint32_t check_sum = std::accumulate(check_data.begin(), check_data.end(), 0u);
  uint8_t expected_checksum = static_cast<uint8_t>(check_sum & 0xFF);

  if (response.back() != expected_checksum) {
    RCLCPP_WARN(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Response checksum mismatch");
  }

  response.pop_back();  // Remove checksum
  return true;
}

bool InspireRH56HandHardwareInterface::send_write_frame(
  uint16_t address, const std::vector<uint8_t> & data)
{
  // Write frame: EB 90 | ID | (Len=DataLen+3) | 0x12 | Addr_L | Addr_H | Data... | Chk
  std::vector<uint8_t> frame = {
    0xEB,
    0x90,                                        // Header
    hand_id_,                                    // Hand ID
    static_cast<uint8_t>(data.size() + 3),       // Length = data + 3 bytes (cmd + addr)
    0x12,                                        // Write command
    static_cast<uint8_t>(address & 0xFF),        // Address low byte
    static_cast<uint8_t>((address >> 8) & 0xFF)  // Address high byte
  };

  // Add data
  frame.insert(frame.end(), data.begin(), data.end());

  // Calculate checksum
  uint32_t sum = std::accumulate(frame.begin() + 2, frame.end(), 0u);
  uint8_t checksum = static_cast<uint8_t>(sum & 0xFF);
  frame.push_back(checksum);

  // Send frame
  if (::write(serial_fd_, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to send write frame");
    return false;
  }

  // Read ACK response: 90 EB | ID | 0x4 | 0x12 | Addr_L | Addr_H | 0x1 | Chk
  std::vector<uint8_t> ack(9);
  if (::read(serial_fd_, ack.data(), 9) != 9) {
    RCLCPP_ERROR(rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to read ACK");
    return false;
  }

  if (ack[0] != 0x90 || ack[1] != 0xEB || ack[2] != hand_id_ || ack[4] != 0x12) {
    RCLCPP_ERROR(rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Invalid ACK response");
    return false;
  }

  return true;
}

std::vector<int16_t> InspireRH56HandHardwareInterface::read_angle_act_registers()
{
  std::vector<uint8_t> response;
  std::vector<int16_t> angles;

  if (!send_read_frame(ANGLE_ACT_ADDR, TOTAL_ANGLE_BYTES, response)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to read ANGLE_ACT");
    return angles;
  }

  if (response.size() != TOTAL_ANGLE_BYTES) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"),
      "Invalid ANGLE_ACT response size: %zu", response.size());
    return angles;
  }

  // Convert little-endian bytes to int16_t values
  angles.resize(NUM_JOINTS);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    uint8_t low_byte = response[i * 2];
    uint8_t high_byte = response[i * 2 + 1];
    angles[i] = static_cast<int16_t>(low_byte | (high_byte << 8));
  }

  return angles;
}

bool InspireRH56HandHardwareInterface::write_angle_set_registers(
  const std::vector<int16_t> & angles)
{
  if (angles.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Invalid angles vector size: %zu",
      angles.size());
    return false;
  }

  // Convert int16_t values to little-endian bytes
  std::vector<uint8_t> data(TOTAL_ANGLE_BYTES);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    data[i * 2] = static_cast<uint8_t>(angles[i] & 0xFF);             // Low byte
    data[i * 2 + 1] = static_cast<uint8_t>((angles[i] >> 8) & 0xFF);  // High byte
  }

  return send_write_frame(ANGLE_SET_ADDR, data);
}

int16_t InspireRH56HandHardwareInterface::position_to_hardware_value(
  double position, size_t joint_index)
{
  // Clamp position to joint limits
  double clamped_pos =
    std::max(joint_min_limits_[joint_index], std::min(joint_max_limits_[joint_index], position));

  // Normalize to 0-1 range
  double range = joint_max_limits_[joint_index] - joint_min_limits_[joint_index];
  double normalized = (clamped_pos - joint_min_limits_[joint_index]) / range;

  // Convert to 0-1000 hardware range with inversion
  return static_cast<int16_t>(1000.0 - normalized * 1000.0);
}

double InspireRH56HandHardwareInterface::hardware_value_to_position(
  int16_t hw_value, size_t joint_index)
{
  // Clamp hardware value to 0-1000 range
  double clamped_hw = std::max(0.0, std::min(1000.0, static_cast<double>(hw_value)));

  // Normalize to 0-1 range with inversion
  double normalized = (1000.0 - clamped_hw) / 1000.0;

  // Convert to joint position range
  double range = joint_max_limits_[joint_index] - joint_min_limits_[joint_index];
  return joint_min_limits_[joint_index] + normalized * range;
}

}  // namespace inspire_rh56_hand_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  inspire_rh56_hand_ros2_control::InspireRH56HandHardwareInterface,
  hardware_interface::SystemInterface)