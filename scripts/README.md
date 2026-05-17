# Workflow Scripts

Run these from PowerShell at the repo root:

```powershell
cd C:\VAULT\armNew
```

Use `powershell -ExecutionPolicy Bypass -File ...` if your execution policy
blocks local scripts.

## Safe Scripts

These do not upload firmware and do not use the XIAO serial port:

```powershell
.\scripts\00_safe_setup_all.ps1
.\scripts\01_build_docker.ps1
.\scripts\02_build_ros_workspace.ps1
.\scripts\03_compile_firmware.ps1
.\scripts\07_display_robot_no_hardware.ps1
```

`07_display_robot_no_hardware.ps1` defaults to a headless URDF/XACRO check so
it works from Windows PowerShell without a Linux GUI. To try RViz, run:

```powershell
.\scripts\07_display_robot_no_hardware.ps1 -Gui
```

RViz requires WSLg or another X11/GUI setup for Docker.
The PowerShell switch is `-Gui` with no space; `- gui` is parsed as a bad
parameter.

## Hardware / Serial Scripts

These touch USB or Serial:

```powershell
.\scripts\04_usb_list.ps1
.\scripts\05_usb_attach_to_wsl.ps1 -BusId <BUSID>
.\scripts\11_check_serial_in_docker.ps1
.\scripts\08_bringup_hardware.ps1 -SerialPort /dev/ttyACM0
.\scripts\09_calibrate_motor.ps1 -Motor 1 -Channel 0
.\scripts\10_safe_halt.ps1
```

Close Arduino Serial Monitor before using hardware scripts.

`05_usb_attach_to_wsl.ps1` must be run from Administrator PowerShell. It
defaults to the `docker-desktop` WSL distro because the ROS 2 tools run inside
Docker. If you only want to inspect the device from Ubuntu, pass
`-Distro Ubuntu`.

Do not run Linux `/dev` commands directly in PowerShell. This is wrong:

```powershell
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Use the checker script instead:

```powershell
.\scripts\11_check_serial_in_docker.ps1
```

The hardware scripts now run that same check before they launch ROS or pyserial,
so a missing `/dev/ttyACM0` should produce a short setup error instead of a ROS
stack trace.

Hardware scripts support `-DryRun` when you want to verify command construction
without touching USB or Serial:

```powershell
.\scripts\08_bringup_hardware.ps1 -DryRun
.\scripts\09_calibrate_motor.ps1 -Motor 1 -Channel 0 -DryRun
.\scripts\10_safe_halt.ps1 -DryRun
```
