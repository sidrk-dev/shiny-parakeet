$ErrorActionPreference = "Stop"

Write-Host "Listing USB devices known to usbipd..." -ForegroundColor Cyan
usbipd list

Write-Host ""
Write-Host "Find the XIAO RP2350 in the list, then run:" -ForegroundColor Yellow
Write-Host "  powershell -ExecutionPolicy Bypass -File .\scripts\05_usb_attach_to_wsl.ps1 -BusId <BUSID>"
Write-Host ""
Write-Host "Important:" -ForegroundColor Yellow
Write-Host "  The ROS Docker container needs the device visible inside Docker's WSL backend."
Write-Host "  This script only lists Windows USB devices; it does not attach anything."
Write-Host ""
Write-Host "To check Linux serial devices from PowerShell, use:" -ForegroundColor Yellow
Write-Host "  powershell -ExecutionPolicy Bypass -File .\scripts\11_check_serial_in_docker.ps1"
