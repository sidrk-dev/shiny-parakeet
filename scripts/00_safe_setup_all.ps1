param(
    [switch]$CleanRos
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

Write-Host "Safe setup: Docker build, ROS workspace build, firmware compile." -ForegroundColor Cyan
Write-Host "This does not upload firmware and does not open Serial." -ForegroundColor Yellow

& "$PSScriptRoot\01_build_docker.ps1"
if ($CleanRos) {
    & "$PSScriptRoot\02_build_ros_workspace.ps1" -Clean
} else {
    & "$PSScriptRoot\02_build_ros_workspace.ps1"
}
& "$PSScriptRoot\03_compile_firmware.ps1"

Write-Host ""
Write-Host "Safe setup complete." -ForegroundColor Green
