param(
    [string]$Command = ""
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

if ([string]::IsNullOrWhiteSpace($Command)) {
    Write-Host "Opening an interactive ROS 2 Docker shell..." -ForegroundColor Cyan
    docker compose -f docker\docker-compose.yml run --rm ros2
} else {
    Write-Host "Running command inside ROS 2 Docker container..." -ForegroundColor Cyan
    docker compose -f docker\docker-compose.yml run --rm ros2 bash -lc $Command
}
