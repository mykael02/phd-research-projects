$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$labPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
$reportPython = Join-Path $projectRoot 'publication\.venv\Scripts\python.exe'
Push-Location $projectRoot
try {
    & $labPython (Join-Path $PSScriptRoot 'run_extension.py') boundary --workers 2
    if ($LASTEXITCODE -ne 0) { throw 'Boundary study failed.' }
    & $labPython (Join-Path $PSScriptRoot 'run_boundary_zoom.py') --workers 2
    if ($LASTEXITCODE -ne 0) { throw 'Boundary zoom failed.' }
    & $labPython (Join-Path $PSScriptRoot 'run_extension.py') payload --workers 2
    if ($LASTEXITCODE -ne 0) { throw 'Payload study failed.' }
    & $labPython (Join-Path $PSScriptRoot 'render_extension_models.py')
    if ($LASTEXITCODE -ne 0) { throw 'Payload rendering failed.' }
    & $labPython (Join-Path $PSScriptRoot 'check_extension_data.py')
    if ($LASTEXITCODE -ne 0) { throw 'Trajectory data audit failed.' }
    & $reportPython (Join-Path $projectRoot 'publication\build_mujoco_extension_report.py')
    if ($LASTEXITCODE -ne 0) { throw 'Extension report failed.' }
} finally {
    Pop-Location
}
