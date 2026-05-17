# RoboMaster Arm ROS 2 Workspace

This workspace contains the first-pass ROS 2 Humble stack for the RP2350
RoboMaster arm controller.

## Packages

### `robomaster_arm_hw`

`ros2_control` hardware interface package.

It provides:

- `robomaster_arm_hw/RoboMasterArmSystemHardware`
- POSIX serial connection to the RP2350.
- Host-side COBS encode/decode.
- Fixed-size binary protocol matching the firmware.
- Position command interfaces.
- Position, velocity, and effort state interfaces.

### `robomaster_arm_description`

URDF/XACRO robot description package.

It provides:

- A generic 6-DOF arm model.
- Dummy cylindrical links.
- Six revolute joints named `joint_1` through `joint_6`.
- A `<ros2_control>` block connected to the hardware plugin.
- Per-joint `motor_id` parameters mapping `joint_1` to Motor ID 1, etc.

### `robomaster_arm_tools`

Calibration and diagnostics package.

It provides beginner-friendly commands that talk to the RP2350 binary protocol:

```bash
ros2 run robomaster_arm_tools config_encoder --motor 1 --channel 0
ros2 run robomaster_arm_tools zero_encoder --motor 1
ros2 run robomaster_arm_tools set_pid --motor 1 --p 0.5 --i 0 --d 0
ros2 run robomaster_arm_tools set_limit --motor 1 --rpm 50
ros2 run robomaster_arm_tools telemetry_once
ros2 run robomaster_arm_tools safe_halt
```

## Build

Inside the Docker container:

```bash
source /opt/ros/humble/setup.bash
cd /workspaces/armNew/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Render The URDF

```bash
ros2 run xacro xacro src/robomaster_arm_description/urdf/robot_arm.urdf.xacro > /tmp/robot_arm.urdf
check_urdf /tmp/robot_arm.urdf
```

## Start A Description-Only RViz Session

```bash
ros2 launch robomaster_arm_description display.launch.py
```

## Start ros2_control Bring-Up

Only run this when the RP2350 firmware is flashed, the serial device is visible
inside the container, and the arm is safe to energize:

```bash
ros2 launch robomaster_arm_description bringup.launch.py serial_port:=/dev/ttyACM0 baud_rate:=921600
```

This starts:

- `robot_state_publisher`
- `controller_manager`
- `joint_state_broadcaster`
- `arm_controller`

## Hardware Bring-Up Sketch

The hardware interface expects the RP2350 firmware in:

```text
../multiMotorUdonTest
```

The default serial settings are:

```text
port: /dev/ttyACM0
baud: 921600
```

The firmware watchdog requires valid command packets at least every 100 ms.
If the ROS 2 controller stops writing commands, the MCU stops all motors.

## Calibration Before Bring-Up

With the firmware uploaded and the XIAO visible as `/dev/ttyACM0` inside Docker:

```bash
ros2 run robomaster_arm_tools set_motor_type --motor 1 --type m3508
ros2 run robomaster_arm_tools config_encoder --motor 1 --channel 0
ros2 run robomaster_arm_tools set_limit --motor 1 --rpm 50
ros2 run robomaster_arm_tools set_pid --motor 1 --p 0.5 --i 0 --d 0
```

Move the joint to the physical zero position, then:

```bash
ros2 run robomaster_arm_tools zero_encoder --motor 1
```

Check one telemetry sample:

```bash
ros2 run robomaster_arm_tools telemetry_once
```

## Control Units

ROS 2 command/state units:

- position: radians
- velocity: radians per second
- effort: currently motor current in amps

Firmware protocol units:

- position: degrees
- velocity: output RPM
- effort/current: milliamps

The hardware interface performs the unit conversions.
