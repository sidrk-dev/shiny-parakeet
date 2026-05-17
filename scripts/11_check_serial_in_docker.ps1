param(
    [string]$SerialPort = "/dev/ttyACM0"
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
. "$PSScriptRoot\_helpers.ps1"

Assert-SerialVisibleInDocker -SerialPort $SerialPort
