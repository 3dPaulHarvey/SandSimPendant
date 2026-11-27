param([string]$Port = '', [switch]$BuildOnly)

$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'upload-firmware.ps1') -Sketch MPU_Visualizer -Port $Port -BuildOnly:$BuildOnly
