param([string]$Port = '', [switch]$BuildOnly)

$ErrorActionPreference = 'Stop'
$projectDirectory = Split-Path -Parent $PSScriptRoot
& (Join-Path $projectDirectory 'upload-firmware.ps1') -Sketch ESP32S3_Diagnostics -Port $Port -BuildOnly:$BuildOnly
