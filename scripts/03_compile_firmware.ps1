$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$SketchDir = Join-Path $RepoRoot "multiMotorUdonTest"

Write-Host "Compiling firmware only. This does not upload and does not open Serial." -ForegroundColor Cyan
Set-Location $SketchDir
& .\compile.cmd
