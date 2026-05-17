param(
    [switch]$Gui
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
. "$PSScriptRoot\_helpers.ps1"

if ($Gui) {
    Write-Host "Launching RViz robot display. This does not connect to hardware." -ForegroundColor Cyan
    Write-Host "This requires a working Linux GUI display from Docker, WSLg, or an X server." -ForegroundColor Yellow
    Assert-DockerGuiAvailable
    docker compose -f docker\docker-compose.yml run --rm ros2 bash -lc `
        "source /opt/ros/humble/setup.bash && source /workspaces/armNew/ros2_ws/install/setup.bash && ros2 launch robomaster_arm_description display.launch.py gui:=true"
} else {
    Write-Host "No GUI requested. Running a headless robot-description check instead." -ForegroundColor Cyan
    Write-Host "Use -Gui from a terminal with WSLg/X11 display support to open RViz." -ForegroundColor Yellow
    $command = "source /opt/ros/humble/setup.bash && source /workspaces/armNew/ros2_ws/install/setup.bash && ros2 run xacro xacro /workspaces/armNew/ros2_ws/src/robomaster_arm_description/urdf/robot_arm.urdf.xacro > /tmp/robot_arm.urdf && test -s /tmp/robot_arm.urdf && grep -q '<robot' /tmp/robot_arm.urdf && echo URDF/XACRO generated successfully"
    docker compose -f docker\docker-compose.yml run --rm ros2 bash -lc $command
}
