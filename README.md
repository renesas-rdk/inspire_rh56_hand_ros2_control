# inspire_rh56_hand_ros2_control

ROS 2 Control hardware interface for the Inspire RH56 6-DOF dexterous hand.

This package provides a ros2_control SystemInterface plugin, URDF/Xacro helpers, controller configs, launch files, and simple examples to command the hand over a serial connection (or a mock for simulation).

## What’s included
- Hardware plugin: `inspire_rh56_hand_ros2_control/InspireRH56HandHardwareInterface`
- Plugin XML: `hardware_interface_plugin.xml`
- URDF/Xacro macro: `urdf/inspire_rh56_hand_ros2_control.xacro`
- Launch files:
  - `inspire_rh56_hand_position_control.launch.py` (group position controller)
  - `inspire_rh56_hand_trajectory_control.launch.py` (joint trajectory controller)
- Controller configs in `config/` and example test scripts in `examples/`

## Dependencies
Declared in `package.xml` (key runtime):
- ros2_control core: `hardware_interface`, `controller_manager`, `joint_state_broadcaster`
- Controllers: `position_controllers`, `joint_trajectory_controller`
- ROS 2: `rclcpp`, `rclcpp_lifecycle`, `pluginlib`, `robot_state_publisher`, `xacro`
- Examples: `rclpy`, `trajectory_msgs`, `control_msgs`

## Build
Standard colcon flow:
```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-select inspire_rh56_hand_ros2_control
source install/setup.bash
```

## URDF integration (ros2_control)
Use the Xacro macro to embed ros2_control into your robot:
```xml
<xacro:include filename="$(find inspire_rh56_hand_ros2_control)/urdf/inspire_rh56_hand_ros2_control.xacro"/>

<xacro:inspire_rh56_hand_ros2_control
  name="inspire_rh56_hand"
  serial_port="/dev/ttyUSB0"
  baudrate="115200"
  use_mock_hardware="false"/>
```
Parameters:
- `serial_port` (string): Serial device path, e.g. `/dev/ttyUSB0`
- `baudrate` (int): 115200 (default)
- `use_mock_hardware` (bool): `true` to simulate with `mock_components/GenericSystem`

The hardware interface exposes 6 joints with position and velocity states:
- `thumb_proximal_yaw_joint`, `thumb_proximal_pitch_joint`,
- `index_proximal_joint`, `middle_proximal_joint`, `ring_proximal_joint`, `pinky_proximal_joint`.

Joint limits are set in the Xacro via command_interface min/max and are used by the hardware interface for clamping and conversion.

## Launch
Position control (direct positions):
```bash
ros2 launch inspire_rh56_hand_ros2_control inspire_rh56_hand_position_control.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200 hand_side:=left use_mock_hardware:=false
```
Trajectory control (FollowJointTrajectory):
```bash
ros2 launch inspire_rh56_hand_ros2_control inspire_rh56_hand_trajectory_control.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200 hand_side:=left use_mock_hardware:=false
```
Arguments:
- `serial_port`, `baudrate` as above
- `hand_side`: `left` or `right` (selects URDF)
- `use_mock_hardware`: `true` to run without a physical hand

Each launch starts:
- `controller_manager` with `controller_manager.yaml`
- `robot_state_publisher`
- `joint_state_broadcaster`
- The selected hand controller (position or trajectory)

## Control interfaces
- Group position controller topic:
  - Publish to: `/inspire_hand_position_controller/commands` (`std_msgs/Float64MultiArray`)
  - Data order: `[thumb_yaw, thumb_pitch, index, middle, ring, pinky]`
- Joint trajectory controller:
  - Action: `/inspire_hand_joint_trajectory_controller/follow_joint_trajectory`
  - Topic: `/inspire_hand_joint_trajectory_controller/joint_trajectory`
  - Joint names must match the list above

## Quick examples
Position command (open/close demo):
```bash
ros2 topic pub /inspire_hand_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Trajectory command (single-point goal):
```bash
ros2 action send_goal /inspire_hand_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [thumb_proximal_yaw_joint, thumb_proximal_pitch_joint, index_proximal_joint, middle_proximal_joint, ring_proximal_joint, pinky_proximal_joint], points: [{positions: [1.0, 0.5, 1.4, 1.4, 1.4, 1.4], time_from_start: {sec: 2}}]}}"
```

## Notes / Troubleshooting
- Serial permissions: ensure your user can access `/dev/ttyUSB*` (e.g., add to the `dialout` group or adjust udev rules).
- Try `use_mock_hardware:=true` to validate controllers and topics without hardware.
- Joint ordering matters when publishing arrays; use the exact order listed above.

## License
Apache-2.0 (see `package.xml`).
