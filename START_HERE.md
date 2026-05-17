# Start Here: RoboMaster Arm Beginner Workflow

This file is the shortest path through the project.

## What You Have

There are two halves:

1. Firmware for the XIAO RP2350:

```text
multiMotorUdonTest/
```

2. ROS 2 workspace for Docker:

```text
ros2_ws/
```

The firmware now uses a binary protocol. The old text commands like `p 1 90`
are intentionally gone.

## Golden Rule

Do the first setup with motor power disconnected.

The scripts below are split into safe scripts and hardware scripts.

Safe scripts:

- build Docker
- build ROS
- compile firmware
- display the robot model

Hardware scripts:

- attach USB to WSL
- configure encoders/PID
- start ros2_control hardware bringup

The normal Docker compose file does not require the XIAO USB device. The
hardware scripts add a second compose file only when they actually need Serial.

## One Command Safe Setup

From PowerShell:

```powershell
cd C:\VAULT\armNew
powershell -ExecutionPolicy Bypass -File .\scripts\00_safe_setup_all.ps1
```

This does:

1. Builds the Docker ROS 2 environment.
2. Builds the ROS 2 workspace inside Docker.
3. Compiles the firmware.

It does not upload firmware and does not open Serial.

## If You Prefer Step By Step

Build Docker:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\01_build_docker.ps1
```

Build ROS:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\02_build_ros_workspace.ps1
```

Compile firmware:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\03_compile_firmware.ps1
```

Open a Docker shell:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\06_enter_ros_container.ps1
```

## Upload Firmware

Upload is still manual on purpose.

Use Arduino IDE:

1. Open:

```text
C:\VAULT\armNew\multiMotorUdonTest\multiMotorUdonTest.ino
```

2. Select the Seeed XIAO RP2350 board.
3. Select the correct port.
4. Upload.
5. Close Arduino Serial Monitor before using ROS tools.

## Attach XIAO USB To WSL/Docker

Open Administrator PowerShell:

```powershell
cd C:\VAULT\armNew
powershell -ExecutionPolicy Bypass -File .\scripts\04_usb_list.ps1
```

Find the XIAO RP2350 BUSID.

Then:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\05_usb_attach_to_wsl.ps1 -BusId <BUSID>
```

This USB attach script must be run from Administrator PowerShell. It attaches
to the `docker-desktop` WSL distro by default because the ROS 2 scripts run in
Docker.

Inside Docker, the board will usually be:

```text
/dev/ttyACM0
```

Check that Docker can see it:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\11_check_serial_in_docker.ps1
```

Do not run Linux commands like `ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null`
directly in PowerShell. PowerShell treats `/dev/null` as `C:\dev\null`.

## Display Robot Without Hardware

This is safe and does not talk to the arm:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\07_display_robot_no_hardware.ps1
```

That default command is headless and should work from normal Windows
PowerShell. RViz needs a Linux GUI display:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\07_display_robot_no_hardware.ps1 -Gui
```

Use `-Gui` with no space. If Docker cannot see a display, the script will stop
before launching RViz.

## Configure Motor 1

Only after firmware is uploaded and the XIAO is attached to WSL/Docker:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\09_calibrate_motor.ps1 -Motor 1 -Channel 0 -MotorType m3508 -P 0.5 -I 0 -D 0 -LimitRpm 50
```

This configures:

- motor type
- encoder channel
- position velocity limit
- position PID

It does not zero by default.

When the joint is physically at the zero position, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\09_calibrate_motor.ps1 -Motor 1 -Channel 0 -MotorType m3508 -P 0.5 -I 0 -D 0 -LimitRpm 50 -Zero
```

## Start Hardware Bringup

Only do this when:

- Firmware is uploaded.
- XIAO is visible as `/dev/ttyACM0`.
- Motor power can be cut quickly.
- The arm is supported.
- You are ready for ROS to send commands.

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\08_bringup_hardware.ps1 -SerialPort /dev/ttyACM0
```

## Emergency Halt Packet

This sends the firmware's binary halt packet:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\10_safe_halt.ps1 -SerialPort /dev/ttyACM0
```

Still keep a physical power cutoff nearby. Software halt is not a substitute
for removing motor power.

## Direct ROS Tool Commands

After building the workspace, you can also run tools manually inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/armNew/ros2_ws/install/setup.bash
ros2 run robomaster_arm_tools config_encoder --motor 1 --channel 0
ros2 run robomaster_arm_tools zero_encoder --motor 1
ros2 run robomaster_arm_tools set_pid --motor 1 --p 0.5 --i 0 --d 0
ros2 run robomaster_arm_tools set_limit --motor 1 --rpm 50
ros2 run robomaster_arm_tools telemetry_once
```
