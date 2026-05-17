param(
    [string]$SerialPort = "/dev/ttyACM0",
    [int]$Baud = 921600,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
. "$PSScriptRoot\_helpers.ps1"

Write-Host "Starting ros2_control hardware bringup." -ForegroundColor Yellow
Write-Host "Only continue if the arm is supported and motor power can be cut quickly." -ForegroundColor Yellow

$command = "source /opt/ros/humble/setup.bash && source /workspaces/armNew/ros2_ws/install/setup.bash && ros2 launch robomaster_arm_description bringup.launch.py serial_port:=$SerialPort baud_rate:=$Baud"
if ($DryRun) {
    Write-Host "DRY RUN: docker compose -f docker\docker-compose.yml -f docker\docker-compose.hardware.yml run --rm ros2 bash -lc `"$command`""
} else {
    Assert-SerialVisibleInDocker -SerialPort $SerialPort
    docker compose -f docker\docker-compose.yml -f docker\docker-compose.hardware.yml run --rm ros2 bash -lc $command
}
