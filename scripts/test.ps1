# Runs all automated tests: JUnit (engine) and cargo test (UI, including the end-to-end engine test).
. "$PSScriptRoot\env.ps1"
Assert-Java21
Push-Location (Join-Path $Root "backend\java-engine")
try {
    & .\gradlew.bat test engineJar --no-daemon
    if ($LASTEXITCODE) { throw "engine tests failed" }
} finally { Pop-Location }
Initialize-Rust
$env:ROBOSBSE_JAVA = Get-JavaExe
Push-Location (Join-Path $Root "frontend\rust-ui")
try {
    cargo test
    if ($LASTEXITCODE) { throw "UI tests failed" }
} finally { Pop-Location }
Write-Host "all tests passed"
