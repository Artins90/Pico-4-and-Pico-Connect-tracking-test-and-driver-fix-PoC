param(
  [double]$HorizonMs = 11.1,
  [string]$LogDir = 'logs',
  [double]$DurationSec = 0
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$exe = Join-Path $root 'build/Pico4VRMotionTest.exe'
if (-not (Test-Path $exe)) { throw "Build not found: $exe" }
$args = @('--horizon-ms', "$HorizonMs", '--log-dir', $LogDir)
if ($DurationSec -gt 0) { $args += @('--duration-s', "$DurationSec") }
& $exe @args
