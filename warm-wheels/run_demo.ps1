# Launch from PowerShell: .\run_demo.ps1
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $demoPython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
    if (-not (Test-Path -LiteralPath $demoPython)) {
        python -m venv .venv
        if ($LASTEXITCODE -ne 0) { throw 'Could not create Python environment.' }
    }
    & $demoPython -m pip install -r requirements.txt
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
    Write-Host 'Open http://127.0.0.1:8000 and paste the session token printed below.'
    & $demoPython -m rover.app --mode demo
} finally {
    Pop-Location
}
