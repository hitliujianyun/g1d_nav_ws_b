# Development build + tests on Windows (MinGW g++ / Ninja / Python 3). DDS executables are not
# built here (unitree_sdk2 ships Linux libraries only); everything else is.
#
#   powershell -ExecutionPolicy Bypass -File scripts\build_host.ps1 [-NoTests] [-Demo]
param([switch]$NoTests, [switch]$Demo)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
$userScripts = Join-Path $env:APPDATA "Python\Python312\Scripts"
if (Test-Path $userScripts) { $env:PATH = "$userScripts;$env:PATH" }

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE) { throw "configure failed" }
cmake --build build -j 8
if ($LASTEXITCODE) { throw "build failed" }

if (-not $NoTests) {
  ctest --test-dir build --output-on-failure
  if ($LASTEXITCODE) { throw "ctest failed" }
  Push-Location python
  try {
    $env:PYTHONPATH = "$Root\python;$Root\third_party\unitree_sdk2_python"
    python -m pytest -q tests
    if ($LASTEXITCODE) { throw "pytest failed" }
  } finally { Pop-Location }
}

if ($Demo) {
  New-Item -ItemType Directory -Force logs | Out-Null
  foreach ($s in "room", "table", "box", "deadend") {
    .\build\src\tools\g1d_sim_demo.exe --scenario $s --out "logs\sim_$s.json"
    python tools\plot_sim_trace.py "logs\sim_$s.json"
  }
}
