param(
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

if ($Clean) {
    Write-Host "Removing ROS build/install/log directories..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue `
        "$RepoRoot\ros2_ws\build", "$RepoRoot\ros2_ws\install", "$RepoRoot\ros2_ws\log"
}

Write-Host "Building ROS 2 workspace inside Docker..." -ForegroundColor Cyan
docker compose -f docker\docker-compose.yml run --rm ros2 bash -lc `
    "source /opt/ros/humble/setup.bash && rosdep update --rosdistro humble && cd /workspaces/armNew/ros2_ws && rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python && colcon build --symlink-install"
