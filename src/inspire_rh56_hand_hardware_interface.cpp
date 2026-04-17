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
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inspire_rh56_hand_ros2_control
{

hardware_interface::CallbackReturn InspireRH56HandHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
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

  // Initialize last_written_angles_ to invalid value to force first write
  last_written_angles_.fill(-1);

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

  // Pre-build the ANGLE_ACT read request frame (constant across all cycles)
  angle_read_frame_[0] = 0xEB;
  angle_read_frame_[1] = 0x90;
  angle_read_frame_[2] = hand_id_;
  angle_read_frame_[3] = 0x04;
  angle_read_frame_[4] = 0x11;
  angle_read_frame_[5] = static_cast<uint8_t>(ANGLE_ACT_ADDR & 0xFF);
  angle_read_frame_[6] = static_cast<uint8_t>((ANGLE_ACT_ADDR >> 8) & 0xFF);
  angle_read_frame_[7] = TOTAL_ANGLE_BYTES;
  uint32_t rd_sum = 0;
  for (size_t i = 2; i < ANGLE_READ_FRAME_SIZE - 1; ++i) {
    rd_sum += angle_read_frame_[i];
  }
  angle_read_frame_[ANGLE_READ_FRAME_SIZE - 1] = static_cast<uint8_t>(rd_sum & 0xFF);

  // Pre-build the ANGLE_SET write frame template (data + checksum updated each cycle)
  angle_write_frame_[0] = 0xEB;
  angle_write_frame_[1] = 0x90;
  angle_write_frame_[2] = hand_id_;
  angle_write_frame_[3] = static_cast<uint8_t>(TOTAL_ANGLE_BYTES + 3);
  angle_write_frame_[4] = 0x12;
  angle_write_frame_[5] = static_cast<uint8_t>(ANGLE_SET_ADDR & 0xFF);
  angle_write_frame_[6] = static_cast<uint8_t>((ANGLE_SET_ADDR >> 8) & 0xFF);

  // Force first write by invalidating last written values
  last_written_angles_.fill(-1);

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
  // Send pre-built ANGLE_ACT read frame (no per-cycle frame construction)
  if (
    ::write(serial_fd_, angle_read_frame_.data(), ANGLE_READ_FRAME_SIZE) !=
    static_cast<ssize_t>(ANGLE_READ_FRAME_SIZE)) {
    RCLCPP_WARN(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to send angle read frame");
    return hardware_interface::return_type::ERROR;
  }

  // Read full response in one robust call (no two-phase header + data reads)
  if (!read_exact(angle_read_resp_buf_.data(), ANGLE_READ_RESP_SIZE)) {
    RCLCPP_WARN(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to read angle response");
    return hardware_interface::return_type::ERROR;
  }

  // Verify response header
  if (
    angle_read_resp_buf_[0] != 0x90 || angle_read_resp_buf_[1] != 0xEB ||
    angle_read_resp_buf_[2] != hand_id_ || angle_read_resp_buf_[4] != 0x11) {
    RCLCPP_WARN(
      rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Invalid angle response header");
    return hardware_interface::return_type::ERROR;
  }

  // Parse angles directly from response (data starts at offset 7, pre-allocated buffer)
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    angle_read_buf_[i] = static_cast<int16_t>(
      angle_read_resp_buf_[7 + i * 2] | (angle_read_resp_buf_[7 + i * 2 + 1] << 8));
  }

  // Store previous positions using pre-allocated buffer (no heap allocation)
  std::copy(hw_positions_.begin(), hw_positions_.end(), prev_positions_.begin());

  // Convert hardware values to joint positions
  for (size_t urdf_idx = 0; urdf_idx < NUM_JOINTS; ++urdf_idx) {
    size_t hw_idx = urdf_to_hw_index(urdf_idx);
    hw_positions_[urdf_idx] = hardware_value_to_position(angle_read_buf_[hw_idx], urdf_idx);
  }

  // Calculate joint velocities using finite difference
  if (first_read_completed_ && period.seconds() > 0.0) {
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      hw_velocities_[i] = (hw_positions_[i] - prev_positions_[i]) / period.seconds();
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
  // Convert URDF joint commands to hardware register values (pre-allocated buffer)
  for (size_t urdf_idx = 0; urdf_idx < NUM_JOINTS; ++urdf_idx) {
    size_t hw_idx = urdf_to_hw_index(urdf_idx);
    angle_write_buf_[hw_idx] = position_to_hardware_value(hw_commands_[urdf_idx], urdf_idx);
  }

  // Skip serial I/O if commands haven't changed since last successful write
  if (angle_write_buf_ != last_written_angles_) {
    // Update data portion of pre-built write frame
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      angle_write_frame_[7 + i * 2] = static_cast<uint8_t>(angle_write_buf_[i] & 0xFF);
      angle_write_frame_[7 + i * 2 + 1] = static_cast<uint8_t>((angle_write_buf_[i] >> 8) & 0xFF);
    }

    // Recalculate checksum (bytes from index 2 to end-1)
    uint32_t sum = 0;
    for (size_t i = 2; i < ANGLE_WRITE_FRAME_SIZE - 1; ++i) {
      sum += angle_write_frame_[i];
    }
    angle_write_frame_[ANGLE_WRITE_FRAME_SIZE - 1] = static_cast<uint8_t>(sum & 0xFF);

    // Send pre-built frame
    if (
      ::write(serial_fd_, angle_write_frame_.data(), ANGLE_WRITE_FRAME_SIZE) !=
      static_cast<ssize_t>(ANGLE_WRITE_FRAME_SIZE)) {
      RCLCPP_WARN(
        rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to send write frame");
      return hardware_interface::return_type::ERROR;
    }

    // Read ACK using robust read (pre-allocated buffer)
    if (!read_exact(angle_write_ack_buf_.data(), ANGLE_WRITE_ACK_SIZE)) {
      RCLCPP_WARN(
        rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Failed to read write ACK");
      return hardware_interface::return_type::ERROR;
    }

    if (
      angle_write_ack_buf_[0] != 0x90 || angle_write_ack_buf_[1] != 0xEB ||
      angle_write_ack_buf_[2] != hand_id_ || angle_write_ack_buf_[4] != 0x12) {
      RCLCPP_WARN(rclcpp::get_logger("InspireRH56HandHardwareInterface"), "Invalid write ACK");
      return hardware_interface::return_type::ERROR;
    }

    // Remember last written values for change detection
    last_written_angles_ = angle_write_buf_;
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

bool InspireRH56HandHardwareInterface::read_exact(uint8_t * buf, size_t count)
{
  size_t total = 0;
  while (total < count) {
    ssize_t n = ::read(serial_fd_, buf + total, count - total);
    if (n <= 0) {
      return false;
    }
    total += static_cast<size_t>(n);
  }
  return true;
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