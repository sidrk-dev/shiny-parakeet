param(
    [switch]$NoCache
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$argsList = @("compose", "-f", "docker\docker-compose.yml", "build")
if ($NoCache) {
    $argsList += "--no-cache"
}

Write-Host "Building the ROS 2 Humble Docker image..." -ForegroundColor Cyan
& docker @argsList
