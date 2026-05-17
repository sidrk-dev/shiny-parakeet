param(
    [int]$Motor = 1,
    [int]$Channel = 0,
    [string]$MotorType = "m3508",
    [double]$P = 0.5,
    [double]$I = 0.0,
    [double]$D = 0.0,
    [double]$LimitRpm = 50.0,
    [string]$SerialPort = "/dev/ttyACM0",
    [switch]$Zero,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
. "$PSScriptRoot\_helpers.ps1"

Write-Host "Configuring motor $Motor on $SerialPort through the binary firmware protocol." -ForegroundColor Cyan
Write-Host "This script uses Serial. Close Arduino Serial Monitor first." -ForegroundColor Yellow

$prefix = "source /opt/ros/humble/setup.bash && source /workspaces/armNew/ros2_ws/install/setup.bash"

$commands = @(
    "ros2 run robomaster_arm_tools set_motor_type --port $SerialPort --motor $Motor --type $MotorType",
    "ros2 run robomaster_arm_tools config_encoder --port $SerialPort --motor $Motor --channel $Channel",
    "ros2 run robomaster_arm_tools set_limit --port $SerialPort --motor $Motor --rpm $LimitRpm",
    "ros2 run robomaster_arm_tools set_pid --port $SerialPort --motor $Motor --p $P --i $I --d $D"
)

if ($Zero) {
    Write-Host "Zeroing motor $Motor at its current physical position." -ForegroundColor Yellow
    $commands += "ros2 run robomaster_arm_tools zero_encoder --port $SerialPort --motor $Motor"
} else {
    Write-Host "Skipped zeroing. Add -Zero when the joint is physically positioned at zero." -ForegroundColor Yellow
}

$commands += "ros2 run robomaster_arm_tools telemetry_once --port $SerialPort"
$joinedCommands = $commands -join " && "
$command = "$prefix && $joinedCommands"
if ($DryRun) {
    Write-Host "DRY RUN: docker compose -f docker\docker-compose.yml -f docker\docker-compose.hardware.yml run --rm ros2 bash -lc `"$command`""
} else {
    Assert-SerialVisibleInDocker -SerialPort $SerialPort
    docker compose -f docker\docker-compose.yml -f docker\docker-compose.hardware.yml run --rm ros2 bash -lc $command
}
