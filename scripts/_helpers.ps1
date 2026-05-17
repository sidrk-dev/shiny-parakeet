$ErrorActionPreference = "Stop"

function Invoke-CheckedNative {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,

        [Parameter(ValueFromRemainingArguments = $true)]
        [string[]]$Arguments
    )

    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$FilePath failed with exit code $LASTEXITCODE"
    }
}

function Assert-SerialVisibleInDocker {
    param(
        [string]$SerialPort = "/dev/ttyACM0"
    )

    if ($SerialPort.Contains("'")) {
        throw "SerialPort must not contain a single quote."
    }

    Write-Host "Checking that $SerialPort is visible inside Docker..." -ForegroundColor Cyan
    docker compose -f docker\docker-compose.yml -f docker\docker-compose.hardware.yml run --rm -e "ARMNEW_SERIAL_PORT=$SerialPort" ros2 bash /workspaces/armNew/scripts/docker_serial_preflight.sh
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

function Assert-DockerGuiAvailable {
    Write-Host "Checking that Docker has a Linux GUI display..." -ForegroundColor Cyan
    docker compose -f docker\docker-compose.yml run --rm ros2 bash /workspaces/armNew/scripts/docker_gui_preflight.sh
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}
