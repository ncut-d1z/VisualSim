$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if (-not $env:VCPKG_ROOT -or -not (Test-Path "$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake")) {
        throw 'Set VCPKG_ROOT to your vcpkg installation first.'
    }

    cmake --preset windows
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }

    cmake --build --preset windows --clean-first
    if ($LASTEXITCODE -ne 0) { throw 'C++ build failed.' }

    ctest --preset windows
    if ($LASTEXITCODE -ne 0) { throw 'CTest failed.' }

    Write-Host ''
    Write-Host 'Build and tests completed successfully.'
    Write-Host 'Executable: out/build/Release/visual_sim.exe'
} finally {
    Pop-Location
}
