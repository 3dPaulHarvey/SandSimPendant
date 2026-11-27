param([string]$Compiler = '')

$ErrorActionPreference = 'Stop'
$projectDirectory = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectDirectory 'build\native-tests'

if (-not $Compiler) {
    $compilerCommand = Get-Command g++ -ErrorAction SilentlyContinue
    if ($compilerCommand) {
        $Compiler = $compilerCommand.Source
    } elseif (Test-Path -LiteralPath 'C:\msys64\ucrt64\bin\g++.exe') {
        $Compiler = 'C:\msys64\ucrt64\bin\g++.exe'
    } else {
        throw 'Install a native C++ compiler or pass -Compiler with the path to g++.'
    }
}

$compilerDirectory = Split-Path -Parent (Get-Command $Compiler -ErrorAction Stop).Source
$previousPath = $env:PATH
try {
    # MinGW executables need the compiler runtime DLLs alongside g++ on PATH.
    $env:PATH = "$compilerDirectory;$previousPath"
    New-Item -ItemType Directory -Path $buildDirectory -Force | Out-Null
    $sources = @(Get-ChildItem -LiteralPath $PSScriptRoot -Filter '*_test.cpp' | Sort-Object Name)
    if ($sources.Count -eq 0) { throw 'No native test sources found.' }
    foreach ($source in $sources) {
        $executable = Join-Path $buildDirectory ($source.BaseName + '.exe')
        Write-Host "Building $($source.Name)"
        & $Compiler -std=c++11 -O2 -Wall -Wextra -Werror -I (Join-Path $PSScriptRoot 'mocks') $source.FullName -o $executable
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($source.Name)" }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $($source.Name) (exit $LASTEXITCODE)" }
    }
    Write-Host "All $($sources.Count) native tests passed."
} finally {
    $env:PATH = $previousPath
}
