# Docker ROS 2 Humble Development Environment

This folder contains a Dockerized ROS 2 Humble environment for developing the
RoboMaster arm hardware interface on a Windows host with WSL2 and Docker
Desktop.

## What The Container Includes

The image is based on:

```text
osrf/ros:humble-desktop
```

It installs:

- `ros2_control`
- `ros2_controllers`
- `controller_manager`
- `hardware_interface`
- MoveIt 2 packages
- RViz 2
- `xacro`
- `robot_state_publisher`
- `joint_state_broadcaster`
- `colcon`
- `rosdep`
- C++ build/debug tools
- Cyclone DDS RMW

## Windows / WSL2 Prerequisites

Install:

1. Docker Desktop for Windows with WSL2 backend enabled.
2. Ubuntu WSL2 distribution.
3. VS Code.
4. VS Code extension: Dev Containers.
5. VS Code extension: Docker.
6. Optional but recommended: `usbipd-win` for USB device forwarding into WSL2.

## USB Serial Access From Windows

Docker containers running under WSL2 cannot automatically see every Windows USB
serial device. The most reliable path is:

1. Connect the XIAO RP2350 over USB.
2. Open PowerShell as Administrator.
3. List USB devices:

```powershell
usbipd list
```

4. Bind the device once:

```powershell
usbipd bind --busid <BUSID>
```

5. Attach it to your WSL distro:

```powershell
usbipd attach --wsl --busid <BUSID>
```

6. In WSL, confirm the port exists:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

The compose file defaults to:

```text
/dev/ttyACM0
```

If your board appears at another path:

```bash
export RP2350_SERIAL=/dev/ttyACM1
```

Then start the container.

The default compose file intentionally does not request a USB device. Hardware
scripts add `docker-compose.hardware.yml` only when Serial access is needed.
This keeps normal build/test steps working even when the XIAO is not attached.

## Build The Container

From the repo root:

```bash
docker compose -f docker/docker-compose.yml build
```

## Run The Container

From the repo root:

```bash
docker compose -f docker/docker-compose.yml run --rm ros2
```

For hardware access, use both compose files:

```bash
docker compose -f docker/docker-compose.yml -f docker/docker-compose.hardware.yml run --rm ros2
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /workspaces/armNew/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Attach VS Code Dev Container

From Windows:

1. Open VS Code.
2. Open the folder `C:\VAULT\armNew`.
3. Press `Ctrl+Shift+P`.
4. Run `Dev Containers: Reopen in Container`.
5. VS Code will use `.devcontainer/devcontainer.json`.

Once inside the container terminal:

```bash
cd /workspaces/armNew/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## ROS 2 Network Notes

The compose file uses:

```yaml
network_mode: host
```

This is the least surprising configuration for ROS 2 DDS discovery on Linux and
WSL2. The default `ROS_DOMAIN_ID` is `42` so this development stack does not
accidentally join another ROS graph using the default domain.

Change it if needed:

```bash
export ROS_DOMAIN_ID=7
```

## Serial Port Parameter

The hardware interface reads the serial port from the URDF ros2_control
hardware parameters.

Default template value:

```text
/dev/ttyACM0
```

Match this to the path visible inside the container.

## Safety Reminder

Do not connect ROS 2 controllers to the powered arm until:

1. The firmware watchdog has been tested.
2. The motor directions are verified.
3. The encoder directions are verified.
4. Position limits are conservative.
5. You can physically remove power quickly.
