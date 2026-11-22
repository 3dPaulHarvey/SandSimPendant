param(
    [string]$Port = 'COM23',
    [switch]$BuildOnly
)

$ErrorActionPreference = 'Stop'
$cliCommand = Get-Command arduino-cli -ErrorAction SilentlyContinue
if ($cliCommand) {
    $arduinoCli = $cliCommand.Source
} else {
    $arduinoCli = Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
    if (-not (Test-Path -LiteralPath $arduinoCli)) {
        throw 'arduino-cli was not found. Install Arduino IDE or put arduino-cli on PATH.'
    }
}

$sketchPath = Join-Path $PSScriptRoot 'Gravity_Sand'
# Keep compiler intermediates outside Dropbox to avoid sync-related file locks.
$buildDirectory = Join-Path $env:LOCALAPPDATA 'SimCircle\build\Gravity_Sand'
$board = 'esp32:esp32:XIAO_ESP32S3:USBMode=hwcdc,CDCOnBoot=default,LoopCore=1'

& $arduinoCli compile --fqbn $board --build-path $buildDirectory $sketchPath
if ($LASTEXITCODE -ne 0) { throw "Compilation failed (exit $LASTEXITCODE); upload skipped." }
if ($BuildOnly) { return }

& $arduinoCli upload --port $Port --fqbn $board --input-dir $buildDirectory $sketchPath
if ($LASTEXITCODE -ne 0) { throw "Upload failed (exit $LASTEXITCODE). Check the port and close Serial Monitor." }
