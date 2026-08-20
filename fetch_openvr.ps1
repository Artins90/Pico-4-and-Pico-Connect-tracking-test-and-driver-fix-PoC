$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$dst = Join-Path $root 'openvr'

if (Test-Path (Join-Path $dst 'headers/openvr.h')) {
  Write-Host "OpenVR already present at $dst"
  exit 0
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
  throw "git.exe was not found. Install Git for Windows or download the ValveSoftware/openvr repository manually."
}

git clone --depth 1 https://github.com/ValveSoftware/openvr.git $dst
Write-Host "OpenVR fetched to $dst"
