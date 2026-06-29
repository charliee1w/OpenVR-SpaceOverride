# Full install validation — run after deploy or package install.
$ErrorActionPreference = "Stop"
$ScriptDir = $PSScriptRoot

& (Join-Path $ScriptDir "smoke-test-steamvr.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& (Join-Path $ScriptDir "debug-steamvr.ps1")
exit $LASTEXITCODE