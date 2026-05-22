// ********************************************************************************************************************
// Copyright [2026] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
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
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "inspire_rh56_hand_ros2_control/visibility_control.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace inspire_rh56_hand_ros2_control
{

class InspireRH56HandHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(InspireRH56HandHardwareInterface)

  INSPIRE_RH56_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  INSPIRE_RH56_HAND_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  INSPIRE_RH56_HAND_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  INSPIRE_RH56_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  INSPIRE_RH56_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  INSPIRE_RH56_HAND_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  INSPIRE_RH56_HAND_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Communication
  std::string serial_port_;
  int serial_fd_;
  int baudrate_;
  uint8_t hand_id_;

  // Protocol constants
  static constexpr uint16_t ANGLE_SET_ADDR = 0x05CE;  // Start address for ANGLE_SET
  static constexpr uint16_t ANGLE_ACT_ADDR = 0x060A;  // Start address for ANGLE_ACT
  static constexpr uint16_t SPEED_SET_ADDR = 0x05F2;  // Start address for SPEED_SET
  static constexpr uint8_t NUM_JOINTS = 6;
  static constexpr uint8_t BYTES_PER_JOINT = 2;  // 16-bit values
  static constexpr uint8_t TOTAL_ANGLE_BYTES = NUM_JOINTS * BYTES_PER_JOINT;

  // Pre-computed frame sizes for control loop I/O
  static constexpr size_t ANGLE_READ_FRAME_SIZE = 9;
  static constexpr size_t ANGLE_READ_RESP_SIZE = 7 + TOTAL_ANGLE_BYTES + 1;
  static constexpr size_t ANGLE_WRITE_FRAME_SIZE = 7 + TOTAL_ANGLE_BYTES + 1;
  static constexpr size_t ANGLE_WRITE_ACK_SIZE = 9;

  // Hardware to URDF joint mapping
  // The hardware registers are in a different order than the URDF joint definitions:
  //
  // Hardware Register Index | Joint Name    | URDF Joint Index
  // -----------------------|---------------|------------------
  //           0            | pinky         |        5
  //           1            | ring          |        4
  //           2            | middle        |        3
  //           3            | index         |        2
  //           4            | thumb_pitch   |        1
  //           5            | thumb_yaw     |        0
  //
  // These mapping arrays provide constant-time translation between the two orderings
  static constexpr std::array<size_t, NUM_JOINTS> URDF_TO_HW_MAP = {5, 4, 3, 2, 1, 0};

  // Joint data
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Configuration
  int hand_speed_;  // Target speed for all joints (0-1000), set once at activation

  // State tracking
  bool first_read_completed_;

  // Joint limits (radians)
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;

  // Pre-allocated control loop buffers (avoid per-cycle heap allocation)
  std::array<int16_t, NUM_JOINTS> angle_read_buf_{};
  std::array<int16_t, NUM_JOINTS> angle_write_buf_{};
  std::array<int16_t, NUM_JOINTS> last_written_angles_{};
  std::array<double, NUM_JOINTS> prev_positions_{};
  std::array<uint8_t, ANGLE_READ_FRAME_SIZE> angle_read_frame_{};
  std::array<uint8_t, ANGLE_READ_RESP_SIZE> angle_read_resp_buf_{};
  std::array<uint8_t, ANGLE_WRITE_FRAME_SIZE> angle_write_frame_{};
  std::array<uint8_t, ANGLE_WRITE_ACK_SIZE> angle_write_ack_buf_{};

  // Helper functions
  bool open_serial_port();
  void close_serial_port();
  int16_t position_to_hardware_value(double position, size_t joint_index);
  double hardware_value_to_position(int16_t hw_value, size_t joint_index);
  bool read_exact(uint8_t * buf, size_t count);

  // Mapping helper function for better code readability
  inline size_t urdf_to_hw_index(size_t urdf_index) const { return URDF_TO_HW_MAP[urdf_index]; }
};

}  // namespace inspire_rh56_hand_ros2_control