param([string]$Port = '', [switch]$BuildOnly)

$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'upload-firmware.ps1') -Sketch Gravity_Sand -Port $Port -BuildOnly:$BuildOnly
