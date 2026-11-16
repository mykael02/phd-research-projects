# Boundary diagnostics, noisy payload repeat and controls, supplement and integrated manuscript.
# Usage, from the project root:
#   powershell -ExecutionPolicy Bypass -File mujoco_lab/run_followup_study.ps1 [-Workers 8]
# Roughly 1 to 2 hours of simulation on a 14-core laptop; results are checkpointed per condition group.
param([int]$Workers = 8)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$labPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
$reportPython = Join-Path $projectRoot 'publication\.venv\Scripts\python.exe'
Push-Location $PSScriptRoot
try {
    & $labPython run_disagreement.py --workers $Workers
    if ($LASTEXITCODE -ne 0) { throw 'Disagreement diagnostics failed.' }
    & $labPython run_disagreement_addendum.py --workers $Workers
    if ($LASTEXITCODE -ne 0) { throw 'Boundary consistency check failed.' }
    & $labPython run_noisy_payload.py --workers $Workers
    if ($LASTEXITCODE -ne 0) { throw 'Noisy payload study failed.' }
    & $labPython run_noisy_controls.py --workers $Workers
    if ($LASTEXITCODE -ne 0) { throw 'Noisy payload controls failed.' }
    & $labPython run_exact_boundary.py --workers $Workers
    if ($LASTEXITCODE -ne 0) { throw 'Exact-feedback boundary study failed.' }
    Set-Location $projectRoot
    & $reportPython (Join-Path $projectRoot 'publication\build_followup_report.py')
    if ($LASTEXITCODE -ne 0) { throw 'Follow-up report failed.' }
    & $reportPython (Join-Path $projectRoot 'publication\build_integrated_manuscript.py')
    if ($LASTEXITCODE -ne 0) { throw 'Integrated manuscript failed.' }
} finally {
    Pop-Location
}
