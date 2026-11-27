param(
    [ValidateSet('MPU_Visualizer', 'Gravity_Sand', 'ESP32S3_Diagnostics')]
    [string]$Sketch = 'MPU_Visualizer',
    [string]$Port = '',
    [switch]$BuildOnly
)

# Pure selection helper: device enumeration is kept outside it for validation.
function Resolve-EspressifPort {
    param([object[]]$Devices, [string[]]$ConnectedPorts)

    $candidates = @(
        foreach ($device in $Devices) {
            if ($device.InstanceId -notmatch '^USB\\VID_303A&') { continue }
            if ($device.FriendlyName -notmatch '\((COM[0-9]+)\)\s*$') { continue }
            $candidate = $Matches[1].ToUpperInvariant()
            if ($ConnectedPorts -contains $candidate) { $candidate }
        }
    ) | Sort-Object -Unique
    $candidates = @($candidates)
    if ($candidates.Count -eq 0) {
        throw 'No connected Espressif USB serial port (VID 303A) was found. Connect the board or pass -Port COMx.'
    }
    if ($candidates.Count -ne 1) {
        throw "Multiple connected Espressif USB serial ports found: $($candidates -join ', '). Pass -Port COMx to select one."
    }
    return $candidates[0]
}

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

$sketchDirectory = Join-Path $PSScriptRoot $Sketch
# Keep build intermediates out of Dropbox; retain logs in the ignored build folder.
$buildDirectory = Join-Path $env:LOCALAPPDATA "SimCircle\build\$Sketch"
$logDirectory = Join-Path $PSScriptRoot "build\$Sketch"
New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
$board = 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,FlashSize=4M,PartitionScheme=default,PSRAM=disabled,FlashMode=dio,CPUFreq=240,LoopCore=1,UploadSpeed=115200'

& $arduinoCli compile --fqbn $board --build-path $buildDirectory $sketchDirectory |
    Tee-Object -FilePath (Join-Path $logDirectory 'compile.log')
if ($LASTEXITCODE -ne 0) { throw "Compilation failed (exit $LASTEXITCODE); upload skipped." }
if ($BuildOnly) { return }

# A replacement board can receive a different COM number. Only choose a board
# automatically when exactly one present Espressif USB serial port is available.
# An explicit -Port is never redirected, including when it is disconnected.
if ([string]::IsNullOrWhiteSpace($Port)) {
    $devices = @(Get-PnpDevice -Class Ports -PresentOnly -ErrorAction Stop)
    $Port = Resolve-EspressifPort -Devices $devices -ConnectedPorts ([System.IO.Ports.SerialPort]::GetPortNames())
    Write-Host "Auto-detected Espressif serial port $Port."
}
if (-not ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port)) {
    throw "$Port is not connected. Reconnect the Super Mini and run this command again."
}

# Use the uploader installed with this board core, not an unrelated Python package.
$detailsJson = & $arduinoCli board details --fqbn $board --format json
if ($LASTEXITCODE -ne 0) { throw 'Could not read ESP32 board settings.' }
$details = $detailsJson | ConvertFrom-Json
$toolProperty = $details.build_properties | Where-Object { $_ -like 'tools.esptool_py.path=*' } | Select-Object -First 1
if (-not $toolProperty) { throw 'Could not locate the ESP32 uploader.' }
$esptool = Join-Path ($toolProperty.Substring($toolProperty.IndexOf('=') + 1)) 'esptool.exe'
if (-not (Test-Path -LiteralPath $esptool)) { throw "Uploader not found: $esptool" }

$bootloader = Join-Path $buildDirectory "$Sketch.ino.bootloader.bin"
$partitions = Join-Path $buildDirectory "$Sketch.ino.partitions.bin"
$bootApp = Join-Path $buildDirectory 'boot_app0.bin'
$application = Join-Path $buildDirectory "$Sketch.ino.bin"
foreach ($binary in @($bootloader, $partitions, $bootApp, $application)) {
    if (-not (Test-Path -LiteralPath $binary)) { throw "Missing compiled binary: $binary" }
}

# Watchdog reset re-samples boot straps after a manual BOOT+RESET upload.
# These offsets match the selected ESP32-S3 default 4MB partition configuration.
& $esptool --chip esp32s3 --port $Port --baud 115200 --after watchdog-reset write-flash `
    --flash-mode dio --flash-freq 80m --flash-size 4MB `
    0x0 $bootloader 0x8000 $partitions 0xe000 $bootApp 0x10000 $application |
    Tee-Object -FilePath (Join-Path $logDirectory 'upload.log')
if ($LASTEXITCODE -ne 0) { throw "Upload failed (exit $LASTEXITCODE). Close Serial Monitor; check USB and BOOT/RESET." }
Write-Host "$Sketch uploaded to $Port. Serial telemetry: 115200 baud."
