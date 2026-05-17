#!/usr/bin/env bash
set -euo pipefail

serial_port="${ARMNEW_SERIAL_PORT:-/dev/ttyACM0}"

if [[ -e "$serial_port" ]]; then
  echo "Serial device visible inside Docker: $serial_port"
  ls -l "$serial_port"
  exit 0
fi

cat >&2 <<EOF
ERROR: $serial_port is not visible inside Docker.

What to do next from Windows PowerShell:
  1. Run: powershell -ExecutionPolicy Bypass -File .\scripts\04_usb_list.ps1
  2. Find the XIAO BUSID.
  3. In Administrator PowerShell, run:
     powershell -ExecutionPolicy Bypass -File .\scripts\05_usb_attach_to_wsl.ps1 -BusId <BUSID>
  4. Re-run this script.

Do not run Linux commands like ls -l /dev/ttyACM* directly in PowerShell.
Use this checker script instead.
EOF

exit 42
