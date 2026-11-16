$ErrorActionPreference = 'Stop'
$labRoot = $PSScriptRoot
$labPython = Join-Path $labRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $labPython)) {
    & python -m venv (Join-Path $labRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Virtual environment creation failed.' }
}
& $labPython -m pip install -r (Join-Path $labRoot 'requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
& $labPython (Join-Path $labRoot 'run.py') --smoke
if ($LASTEXITCODE -ne 0) { throw 'MuJoCo setup check failed.' }
