$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) { throw 'cmake.exe not found on PATH.' }
if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) { throw 'ninja.exe not found on PATH.' }
if (-not (Get-Command clang-cl -ErrorAction SilentlyContinue)) { throw 'clang-cl.exe not found on PATH.' }
if (-not (Get-Command lld-link -ErrorAction SilentlyContinue)) { throw 'lld-link.exe not found on PATH. Add the LLVM bin directory to PATH.' }

if (-not (Test-Path 'openvr/headers/openvr.h')) {
  & "$PSScriptRoot/fetch_openvr.ps1"
}

cmake -S . -B build -G Ninja `
  -DCMAKE_C_COMPILER=clang-cl `
  -DCMAKE_CXX_COMPILER=clang-cl `
  -DCMAKE_LINKER=lld-link

cmake --build build
Write-Host "Built: $root\build\Pico4VRMotionTest.exe"
