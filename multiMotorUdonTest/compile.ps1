$ErrorActionPreference = "Stop"

$SketchDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Cli = "C:\Users\sidth\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$Fqbn = "rp2040:rp2040:seeed_xiao_rp2350"

if (!(Test-Path $Cli)) {
    $Cli = "arduino-cli"
}

& $Cli compile --fqbn $Fqbn $SketchDir
