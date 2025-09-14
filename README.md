# inspire_rh56_hand_ros2_control

ROS 2 package that provides a ros2_control hardware interface for the Inspire RH56 6-DOF dexterous hand. This package contains the hardware interface implementation that communicates with the physical hand hardware via serial connection.

## Features
- ros2_control hardware interface plugin (exported via `hardware_interface_plugin.xml`)
- Hardware interface implementation for serial communication with the hand
- ros2_control URDF macro for hardware interface integration

## Related Packages
- **inspire_rh56_hand_bringup**: Contains launch files, controller configurations, robot URDF descriptions, and test scripts for operating the hand
- **inspire_rh56_hand_description**: Contains the hand's visual and collision meshes, joint definitions

## Package layout
- `include/` / `src/`: Hardware interface implementation (`inspire_rh56_hand_hardware_interface`)
- `urdf/`: ros2_control URDF macro for hardware interface integration
- `hardware_interface_plugin.xml`: Plugin description file for the hardware interface

## Prerequisites
- ROS 2 (Jazzy or newer) with `ros2_control` ecosystem
- A colcon workspace (e.g., `~/ros2_ws`)
- Serial interface support for hardware communication

## Usage
This package provides the hardware interface that should be used with the `inspire_rh56_hand_bringup` package for launching and controlling the hand.

To use this hardware interface in your hand system, you'll typically launch one of the bringup configurations:

```bash
# For joint position control
ros2 launch inspire_rh56_hand_bringup inspire_rh56_hand_joint_position_control.launch.py

# For joint trajectory control  
ros2 launch inspire_rh56_hand_bringup inspire_rh56_hand_joint_trajectory_control.launch.py
```

After launching, you can introspect the hardware interface:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

## URDF Integration
The ros2_control hardware interface macro is provided in `urdf/`:
- `inspire_rh56_hand_macro.ros2_control.xacro`: ros2_control hardware interface, transmissions, and interfaces

To include the hardware interface in your hand URDF:
```xml
<xacro:include filename="$(find inspire_rh56_hand_ros2_control)/urdf/inspire_rh56_hand_macro.ros2_control.xacro"/>
<xacro:inspire_rh56_hand_ros2_control
  name="inspire_rh56_hand"
  serial_port="/dev/ttyUSB0"
  baudrate="115200"
  use_mock_hardware="false"/>
```
Adjust arguments as needed (see the xacro file for available parameters).

## Configuration Options
The hardware interface supports the following configuration options:
- `serial_port`: Serial device path for hardware communication (e.g., "/dev/ttyUSB0")
- `baudrate`: Serial communication baudrate (default: "115200")
- `use_mock_hardware`: Set to "true" for simulation/testing without physical hardware

## Joint Interfaces
The hardware interface exposes 6 joints with position and velocity states:
- `thumb_proximal_yaw_joint`, `thumb_proximal_pitch_joint`
- `index_proximal_joint`, `middle_proximal_joint`, `ring_proximal_joint`, `pinky_proximal_joint`

## Development
For detailed usage examples, launch configurations, controller setups, and test scripts, see the `inspire_rh56_hand_bringup` package.
