# Starts the RoboSBSE desktop UI (builds it first if needed). Extra arguments are passed to the UI,
# e.g.  scripts\run.ps1 --page testing
. "$PSScriptRoot\env.ps1"
Assert-Java21
$env:ROBOSBSE_JAVA = Get-JavaExe
$jar = Join-Path $Root "backend\java-engine\build\libs\robosbse-engine.jar"
$exe = Join-Path $Root "frontend\rust-ui\target\release\robosbse.exe"
if (-not (Test-Path $exe)) { $exe = Join-Path $Root "frontend\rust-ui\target\debug\robosbse.exe" }
if (-not (Test-Path $jar) -or -not (Test-Path $exe)) {
    & "$PSScriptRoot\build.ps1" -Release
    $exe = Join-Path $Root "frontend\rust-ui\target\release\robosbse.exe"
}
Push-Location $Root
try { & $exe @args } finally { Pop-Location }
