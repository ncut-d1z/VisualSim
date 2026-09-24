param(
  [string]$OutputDir = 'datasets'
)

$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $exe = Join-Path $PSScriptRoot 'out/build/Release/visual_sim.exe'

    if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) {
        Write-Host 'visual_sim.exe was not found. Building the project first...'
        & (Join-Path $PSScriptRoot 'build.ps1')
        if ($LASTEXITCODE -ne 0) {
            throw 'build.ps1 failed.'
        }
    }

    if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) {
        throw "Executable was not produced: $exe"
    }

    $datasetPath = if ([System.IO.Path]::IsPathRooted($OutputDir)) {
        [System.IO.Path]::GetFullPath($OutputDir)
    } else {
        [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot $OutputDir))
    }

    Write-Host ''
    Write-Host "Running: $exe"
    Write-Host "Dataset output: $datasetPath"

    & $exe $datasetPath
    if ($LASTEXITCODE -ne 0) {
        throw "visual_sim.exe failed with exit code $LASTEXITCODE."
    }

    if (-not (Test-Path -LiteralPath $datasetPath -PathType Container)) {
        throw "Simulation completed, but the dataset directory was not created: $datasetPath"
    }

    $resolvedDatasetPath = (Resolve-Path -LiteralPath $datasetPath).Path
    Write-Host ''
    Write-Host 'Simulation datasets generated successfully.'
    Write-Host "Absolute dataset path: $resolvedDatasetPath"
} finally {
    Pop-Location
}
