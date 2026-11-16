$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
Push-Location $projectRoot
try {
    & (Join-Path $PSScriptRoot '.venv\Scripts\python.exe') (Join-Path $PSScriptRoot 'check_reference.py')
    if ($LASTEXITCODE -ne 0) { throw 'Original simulation entry-point check failed.' }
    & (Join-Path $PSScriptRoot '.venv\Scripts\python.exe') (Join-Path $PSScriptRoot 'validate.py')
    if ($LASTEXITCODE -ne 0) { throw 'MuJoCo validation run failed.' }
    & (Join-Path $projectRoot 'publication\.venv\Scripts\python.exe') (Join-Path $projectRoot 'publication\build_mujoco_report.py')
    if ($LASTEXITCODE -ne 0) { throw 'MuJoCo report generation failed.' }
} finally {
    Pop-Location
}
