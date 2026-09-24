param(
  [string]$BuildDir = 'out/build'
)

$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
  $root = (Resolve-Path .).Path
  $buildPath = Join-Path $PSScriptRoot $BuildDir

  if (-not (Test-Path $buildPath)) {
    Write-Host "Nothing to clean: $BuildDir does not exist."
    exit 0
  }

  $resolvedBuild = (Resolve-Path $buildPath).Path
  if (-not $resolvedBuild.StartsWith($root, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to clean outside project: $resolvedBuild"
  }

  Remove-Item -LiteralPath $resolvedBuild -Recurse -Force
  Write-Host "Removed build directory: $resolvedBuild"
} finally {
  Pop-Location
}
