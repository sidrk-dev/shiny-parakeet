param(
    [Parameter(Mandatory = $true)]
    [string]$BusId,

    [string]$Distro = "docker-desktop",

    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
. "$PSScriptRoot\_helpers.ps1"

function Invoke-Usbipd {
    param(
        [Parameter(ValueFromRemainingArguments = $true)]
        [string[]]$Arguments
    )

    & usbipd @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "usbipd $($Arguments -join ' ') failed with exit code $LASTEXITCODE"
    }
}

Write-Host "Binding USB device $BusId. This may require Administrator PowerShell." -ForegroundColor Cyan
if ($DryRun) {
    Write-Host "DRY RUN: usbipd bind --busid $BusId"
} else {
    try {
        Invoke-Usbipd @("bind", "--busid", $BusId)
    } catch {
        Write-Host ""
        Write-Host "USB bind failed." -ForegroundColor Red
        Write-Host "Run this script from an Administrator PowerShell window." -ForegroundColor Yellow
        Write-Host "If usbipd says the device is already shared, that is okay; run 04_usb_list.ps1 to confirm." -ForegroundColor Yellow
        exit 1
    }
}

Write-Host "Starting WSL distro '$Distro' so usbipd has a running target..." -ForegroundColor Cyan
if ($DryRun) {
    Write-Host "DRY RUN: wsl -d $Distro --exec sh -lc `"nohup sleep 86400 >/tmp/armnew-usbipd-keepalive.log 2>&1 &`""
} else {
    try {
        Invoke-CheckedNative wsl -d $Distro --exec sh -lc "nohup sleep 86400 >/tmp/armnew-usbipd-keepalive.log 2>&1 &"
    } catch {
        Write-Host ""
        Write-Host "Could not start WSL distro '$Distro'." -ForegroundColor Red
        Write-Host "Make sure Docker Desktop is running. If you want Ubuntu instead, rerun with -Distro Ubuntu." -ForegroundColor Yellow
        exit 1
    }
}

Write-Host "Attaching USB device $BusId to WSL distro '$Distro'..." -ForegroundColor Cyan
if ($DryRun) {
    Write-Host "DRY RUN: usbipd attach --wsl --distribution $Distro --busid $BusId"
} else {
    try {
        Invoke-Usbipd @("attach", "--wsl", "--distribution", $Distro, "--busid", $BusId)
    } catch {
        Write-Host ""
        Write-Host "USB attach failed." -ForegroundColor Red
        Write-Host "Make sure Docker Desktop is running and that this PowerShell window is Administrator." -ForegroundColor Yellow
        Write-Host "If you intentionally want Ubuntu instead of Docker, rerun with -Distro Ubuntu." -ForegroundColor Yellow
        exit 1
    }
}

Write-Host ""
Write-Host "Checking serial devices inside $Distro..." -ForegroundColor Cyan
if ($DryRun) {
    Write-Host "DRY RUN: wsl -d $Distro --exec sh -lc `"ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true`""
} else {
    try {
        Invoke-CheckedNative wsl -d $Distro --exec sh -lc "ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true"
    } catch {
        Write-Host "Could not list serial devices inside '$Distro'." -ForegroundColor Red
        exit 1
    }
}

Write-Host ""
Write-Host "Next check from Docker:" -ForegroundColor Yellow
Write-Host "  powershell -ExecutionPolicy Bypass -File .\scripts\11_check_serial_in_docker.ps1"
