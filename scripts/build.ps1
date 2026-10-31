# Builds the Java engine jar and the Rust desktop UI.  Usage: scripts\build.ps1 [-Release]
param([switch]$Release)
. "$PSScriptRoot\env.ps1"
Assert-Java21
Push-Location (Join-Path $Root "backend\java-engine")
try {
    & .\gradlew.bat engineJar --no-daemon -q
    if ($LASTEXITCODE) { throw "engine build failed" }
} finally { Pop-Location }
Initialize-Rust
Push-Location (Join-Path $Root "frontend\rust-ui")
try {
    if ($Release) { cargo build --release } else { cargo build }
    if ($LASTEXITCODE) { throw "UI build failed" }
} finally { Pop-Location }
$profile_ = if ($Release) { "release" } else { "debug" }
Write-Host "built backend\java-engine\build\libs\robosbse-engine.jar and frontend\rust-ui\target\$profile_\robosbse.exe"
