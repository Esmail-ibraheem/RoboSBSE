# Shared environment for the PowerShell scripts: repository root, Java 21 and Rust toolchain checks.
# Machine-specific settings (e.g. a portable JDK) go into scripts/env.local.ps1, which is not committed:
#   $env:JAVA_HOME = "C:\path\to\jdk-21"
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$local = Join-Path $PSScriptRoot "env.local.ps1"
if (Test-Path $local) { . $local }

function Get-JavaExe {
    if ($env:ROBOSBSE_JAVA) { return $env:ROBOSBSE_JAVA }
    if ($env:JAVA_HOME -and (Test-Path (Join-Path $env:JAVA_HOME "bin\java.exe"))) {
        return (Join-Path $env:JAVA_HOME "bin\java.exe")
    }
    return "java"
}

function Assert-Java21 {
    $java = Get-JavaExe
    $text = (& cmd /c "`"$java`" -version 2>&1") -join " "
    if ($text -notmatch 'version "(\d+)') {
        throw "Java not found ($java). Install JDK 21+ or set JAVA_HOME (scripts/env.local.ps1)."
    }
    if ([int]$Matches[1] -lt 21) { throw "Java $($Matches[1]) found at $java; RoboSBSE needs Java 21+. Set JAVA_HOME." }
    if (-not $env:JAVA_HOME -and $java -ne "java") {
        $env:JAVA_HOME = Split-Path -Parent (Split-Path -Parent $java)
    }
}

function Initialize-Rust {
    $cargoBin = Join-Path $HOME ".cargo\bin"
    if (Test-Path $cargoBin) { $env:PATH = "$cargoBin;$env:PATH" }
    if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) { throw "cargo not found. Install Rust from https://rustup.rs" }
    # The windows-gnu toolchain needs GNU binutils (dlltool + as) for crates that use raw-dylib.
    $hostLine = (& rustc -vV | Select-String "host:").ToString()
    if ($hostLine -match "windows-gnu" -and -not (Get-Command as -ErrorAction SilentlyContinue)) {
        foreach ($dir in @("C:\msys64\mingw64\bin", "C:\msys64\ucrt64\bin")) {
            if (Test-Path (Join-Path $dir "as.exe")) { $env:PATH = "$env:PATH;$dir"; break }
        }
    }
}
