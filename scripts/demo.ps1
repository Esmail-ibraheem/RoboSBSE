# Headless demonstration of the Phase 1 vertical slice (no UI): route optimisation, automatic
# discovery of an injected controller defect, and reproduction of the saved failure record.
. "$PSScriptRoot\env.ps1"
Assert-Java21
$java = Get-JavaExe
$jar = Join-Path $Root "backend\java-engine\build\libs\robosbse-engine.jar"
if (-not (Test-Path $jar)) { & "$PSScriptRoot\build.ps1" }
Push-Location $Root
try {
    & $java -jar $jar run examples\experiments\nav-warehouse-ga.json --out experiments\results
    & $java -jar $jar run examples\experiments\testing-bug1-ga.json --out experiments\results
    $latest = Get-ChildItem experiments\results -Directory | Where-Object Name -like "*testing-bug1-ga*" |
        Sort-Object Name | Select-Object -Last 1
    & $java -jar $jar reproduce (Join-Path $latest.FullName "failures\failure-1.json")
} finally { Pop-Location }
