param(
    [string]$SerialPort = "/dev/ttyACM0",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
. "$PSScriptRoot\_helpers.ps1"

Write-Host "Sending a binary halt packet to the RP2350." -ForegroundColor Yellow
$command = "source /opt/ros/humble/setup.bash && source /workspaces/armNew/ros2_ws/install/setup.bash && ros2 run robomaster_arm_tools safe_halt --port $SerialPort"
if ($DryRun) {
    Write-Host "DRY RUN: docker compose -f docker\docker-compose.yml -f docker\docker-compose.hardware.yml run --rm ros2 bash -lc `"$command`""
} else {
    Assert-SerialVisibleInDocker -SerialPort $SerialPort
    docker compose -f docker\docker-compose.yml -f docker\docker-compose.hardware.yml run --rm ros2 bash -lc $command
}
