# Mouse Whisker - Release Build Script
# Compiles firmware binaries for all supported ESP32 boards
#
# Usage: .\build-releases.ps1
#        .\build-releases.ps1 -Version "1.5.1"
#
# Output: Creates 'releases' folder with .bin files for GitHub Releases
#
# ============================================================================
# PREREQUISITES
# ============================================================================
#
# 1. Install arduino-cli
#    Download: https://arduino.github.io/arduino-cli/latest/installation/
#    Windows:  winget install ArduinoSA.CLI
#              -or- choco install arduino-cli
#              -or- download from https://github.com/arduino/arduino-cli/releases
#
# 2. Add ESP32 board support
#    arduino-cli config init
#    arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
#    arduino-cli core update-index
#    arduino-cli core install esp32:esp32
#
# 3. Install required libraries
#    arduino-cli lib install "NimBLE-Arduino"
#    arduino-cli lib install "PubSubClient"
#
# ============================================================================

param(
    [string]$Version = ""
)

$ErrorActionPreference = "Stop"

# Extract version from source if not provided
if (-not $Version) {
    $sourceFile = Join-Path $PSScriptRoot "mouse-whisker.ino"
    $content = Get-Content $sourceFile -Raw
    if ($content -match '#define FIRMWARE_VERSION "([^"]+)"') {
        $Version = $matches[1]
        Write-Host "Detected version: $Version" -ForegroundColor Cyan
    } else {
        Write-Error "Could not detect version from source. Use -Version parameter."
        exit 1
    }
}

# Build configurations
# Format: Name, FQBN, Build flags, Output suffix
# Flash limits are extracted from compiler output (not hardcoded)
# All boards use min_spiffs partition scheme for more app space (~1.9MB vs 1.2MB)
$builds = @(
    @{
        Name = "ESP32-C3 (BLE)"
        FQBN = "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=min_spiffs"
        Flags = ""
        Suffix = "esp32c3_ble"
    },
    @{
        Name = "ESP32-S3 (BLE + USB)"
        FQBN = "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=cdc,PartitionScheme=min_spiffs"
        Flags = ""
        Suffix = "esp32s3_ble_usb"
        # Note: Requires ENABLE_USB=true in sketch (default is false)
        # This build is for users who want USB HID
        SketchOverrides = @{ "ENABLE_USB" = "true" }
    },
    @{
        Name = "ESP32-S3 (BLE only)"
        FQBN = "esp32:esp32:esp32s3:CDCOnBoot=cdc,PartitionScheme=min_spiffs"
        Flags = ""
        Suffix = "esp32s3_ble"
    },
    @{
        Name = "ESP32-S2 (USB)"
        FQBN = "esp32:esp32:esp32s2:CDCOnBoot=cdc,PartitionScheme=min_spiffs"
        Flags = ""
        Suffix = "esp32s2_usb"
        SketchOverrides = @{ "ENABLE_USB" = "true"; "ENABLE_BLE" = "false" }
    },
    @{
        Name = "ESP32 Classic (BLE)"
        FQBN = "esp32:esp32:esp32:PartitionScheme=min_spiffs"
        Flags = ""
        Suffix = "esp32_ble"
    }
)

# Paths
$sketchPath = Join-Path $PSScriptRoot "mouse-whisker.ino"
$releaseDir = Join-Path $PSScriptRoot "releases"
$tempDir = Join-Path $PSScriptRoot "build-temp"

# Clean up any leftover temp files from previous runs
if (Test-Path $tempDir) {
    Remove-Item -Path $tempDir -Recurse -Force -ErrorAction SilentlyContinue
}

# Create output directories
if (-not (Test-Path $releaseDir)) {
    New-Item -ItemType Directory -Path $releaseDir | Out-Null
}
New-Item -ItemType Directory -Path $tempDir | Out-Null

# Verify arduino-cli is available
try {
    $null = & arduino-cli version
} catch {
    Write-Error "arduino-cli not found. Please install it and ensure it's in PATH."
    Write-Host "Install with: winget install ArduinoSA.ArduinoCLI" -ForegroundColor Yellow
    exit 1
}

Write-Host "`n========================================" -ForegroundColor Green
Write-Host " Mouse Whisker Release Builder v$Version" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Green

$successCount = 0
$failCount = 0
$results = @()

foreach ($build in $builds) {
    $outputName = "mouse-whisker_v${Version}_$($build.Suffix)_full.bin"
    $outputPath = Join-Path $releaseDir $outputName
    $buildDir = Join-Path $tempDir $build.Suffix
    
    Write-Host "Building: $($build.Name)" -ForegroundColor Yellow
    Write-Host "  FQBN: $($build.FQBN)" -ForegroundColor Gray
    
    # Prepare build command
    $buildArgs = @(
        "compile"
        "--fqbn", $build.FQBN
        "--output-dir", $buildDir
        "--warnings", "none"
    )
    
    # Add verbose flag for progress output (unless -Quiet)
    if (-not $Quiet) {
        $buildArgs += "-v"
    }
    
    # Add build flags if present
    if ($build.Flags) {
        $buildArgs += "--build-property"
        $buildArgs += "build.extra_flags=$($build.Flags)"
        Write-Host "  Flags: $($build.Flags)" -ForegroundColor Gray
    }
    
    # Handle sketch overrides (create temp modified sketch)
    # Arduino requires folder name to match .ino filename
    $sketchToCompile = $sketchPath
    if ($build.SketchOverrides) {
        $tempSketchDir = Join-Path $tempDir "$($build.Suffix)-sketch"
        $tempSketchSubDir = Join-Path $tempSketchDir "mouse-whisker"
        $tempSketch = Join-Path $tempSketchSubDir "mouse-whisker.ino"
        
        if (-not (Test-Path $tempSketchSubDir)) {
            New-Item -ItemType Directory -Path $tempSketchSubDir -Force | Out-Null
        }
        
        $sketchContent = Get-Content $sketchPath -Raw
        foreach ($key in $build.SketchOverrides.Keys) {
            $value = $build.SketchOverrides[$key]
            # Match #define with optional leading whitespace
            $sketchContent = $sketchContent -replace "(\s*#define\s+$key\s+)(true|false)", "`$1$value"
            Write-Host "  Override: $key = $value" -ForegroundColor Gray
        }
        $sketchContent | Set-Content $tempSketch -NoNewline
        $sketchToCompile = $tempSketch
    }
    
    $buildArgs += $sketchToCompile
    
    # Run build with spinner
    $startTime = Get-Date
    
    try {
        Write-Host "  Compiling... " -NoNewline -ForegroundColor Gray
        
        # Build the full command line (sketch path already in buildArgs)
        $argString = $buildArgs -join ' '
            
            # Start process with async output capture
            $psi = New-Object System.Diagnostics.ProcessStartInfo
            $psi.FileName = "arduino-cli"
            $psi.Arguments = $argString
            $psi.UseShellExecute = $false
            $psi.RedirectStandardOutput = $true
            $psi.RedirectStandardError = $true
            $psi.CreateNoWindow = $true
            
            $process = New-Object System.Diagnostics.Process
            $process.StartInfo = $psi
            $process.Start() | Out-Null
            
            # Begin async reads to prevent buffer deadlock
            $stdoutTask = $process.StandardOutput.ReadToEndAsync()
            $stderrTask = $process.StandardError.ReadToEndAsync()
            
            # Spinner animation while process runs
            $spinChars = @('|', '/', '-', '\')
            $spinIdx = 0
            
            while (-not $process.HasExited) {
                $elapsed = [math]::Round(((Get-Date) - $startTime).TotalSeconds, 0)
                Write-Host "`r  Compiling... $($spinChars[$spinIdx]) ($elapsed`s)  " -NoNewline -ForegroundColor Gray
                $spinIdx = ($spinIdx + 1) % 4
                Start-Sleep -Milliseconds 150
            }
            
            # Wait for async reads to complete and get output
            $stdout = $stdoutTask.Result
            $stderr = $stderrTask.Result
            $exitCode = $process.ExitCode
            $outputText = $stdout + "`n" + $stderr
            $output = $outputText -split "`n"
            
            # Clear the spinner line
            $elapsed = [math]::Round(((Get-Date) - $startTime).TotalSeconds, 1)
            Write-Host "`r  Compiling... done ($elapsed`s)        " -ForegroundColor Gray
        
        if ($exitCode -eq 0) {
            # Parse flash usage from compiler output
            # Example: "Sketch uses 892456 bytes (68%) of program storage space. Maximum is 1310720 bytes."
            $maxAppSize = 0
            $usedBytes = 0
            if ($outputText -match 'Sketch uses (\d+) bytes \((\d+)%\) of program storage space\. Maximum is (\d+) bytes') {
                $usedBytes = [int]$matches[1]
                $maxAppSize = [int]$matches[3]
            }
            
            # Find both binary files:
            # - merged.bin = full flash image (bootloader + partition + app) for initial USB flashing
            # - .ino.bin = app only, for OTA updates
            $mergedBin = Get-ChildItem -Path $buildDir -Filter "*.ino.merged.bin" -ErrorAction SilentlyContinue | Select-Object -First 1
            $appBin = Get-ChildItem -Path $buildDir -Filter "*.ino.bin" -ErrorAction SilentlyContinue | Select-Object -First 1
            
            # Use merged for main release (USB flashing), app-only for OTA
            $binFile = if ($mergedBin) { $mergedBin } else { $appBin }
            
            if ($binFile) {
                # Copy main binary (merged for USB flashing)
                Copy-Item $binFile.FullName $outputPath -Force
                $fileSize = (Get-Item $outputPath).Length
                
                # Also copy app-only binary for OTA updates (if we have merged, then app is separate)
                if ($mergedBin -and $appBin) {
                    $otaOutputName = "mouse-whisker_v${Version}_$($build.Suffix)_ota.bin"
                    $otaOutputPath = Join-Path $releaseDir $otaOutputName
                    Copy-Item $appBin.FullName $otaOutputPath -Force
                    $otaFileSize = (Get-Item $otaOutputPath).Length
                    $otaFileSizeKB = [math]::Round($otaFileSize / 1024, 1)
                }
                
                $sizeDisplay = $fileSize.ToString("N0")  # Format with thousands separator
                
                # Use compiler-reported max if available, otherwise estimate from file
                if ($maxAppSize -gt 0) {
                    $maxDisplay = $maxAppSize.ToString("N0")
                    # Use usedBytes from compiler for accurate percentage (excludes bootloader overhead in merged bin)
                    $percent = [math]::Round(($usedBytes / $maxAppSize) * 100, 1)
                } else {
                    # Fallback: assume 1.25MB default partition
                    $maxAppSize = 1310720
                    $maxDisplay = $maxAppSize.ToString("N0")
                    $percent = [math]::Round(($fileSize / $maxAppSize) * 100, 1)
                    Write-Host "  Warning: Could not parse compiler output, using default 1.25MB limit" -ForegroundColor Yellow
                }
                
                # Determine color based on usage
                # Green: <75%, Yellow: 75-90%, DarkYellow: 90%+, Red: >=100%
                $usageColor = if ($percent -ge 100) { "Red" } 
                              elseif ($percent -ge 90) { "DarkYellow" } 
                              elseif ($percent -ge 75) { "Yellow" } 
                              else { "Green" }
                
                if ($percent -ge 100) {
                    $errMsg = "  X TOO LARGE: $outputName (" + $usedBytes.ToString("N0") + " / " + $maxAppSize.ToString("N0") + " bytes = " + $percent + "%)"
                    Write-Host $errMsg -ForegroundColor Red
                    $failCount++
                    $fileSizeKB = [math]::Round($fileSize / 1024, 1)
                    $results += @{ Name = $build.Name; Status = "Too Large"; File = $outputName; FileSizeKB = $fileSizeKB; SketchBytes = $usedBytes; MaxBytes = $maxAppSize; Percent = $percent }
                } else {
                    Write-Host "  OK Success" -ForegroundColor Green
                    $sizeMsg = "    Sketch: " + $usedBytes.ToString("N0") + " / " + $maxAppSize.ToString("N0") + " bytes (" + $percent + "% of flash)"
                    Write-Host $sizeMsg -ForegroundColor $usageColor
                    $fileSizeKB = [math]::Round($fileSize / 1024, 1)
                    Write-Host "    Full: $fileSizeKB KB ($outputName)" -ForegroundColor Gray
                    if ($mergedBin -and $appBin) {
                        Write-Host "    OTA:  $otaFileSizeKB KB ($otaOutputName)" -ForegroundColor Gray
                    }
                    $successCount++
                    $results += @{ Name = $build.Name; Status = "Success"; File = $outputName; FileSizeKB = $fileSizeKB; SketchBytes = $usedBytes; MaxBytes = $maxAppSize; Percent = $percent }
                }
                
                # Clean up build directory after copying .bin (saves disk space)
                if (Test-Path $buildDir) {
                    Remove-Item -Path $buildDir -Recurse -Force -ErrorAction SilentlyContinue
                }
            } else {
                Write-Host "  ✗ Build succeeded but .bin not found" -ForegroundColor Red
                $failCount++
                $results += @{ Name = $build.Name; Status = "Failed"; File = ""; Size = ""; MaxKB = 0; Percent = 0 }
            }
        } else {
            Write-Host "  ✗ Build failed (exit code: $exitCode)" -ForegroundColor Red
            # Show size info if available (useful for "sketch too big" errors)
            $sizeLines = $output | Where-Object { $_ -match "Sketch uses|Maximum is|text section|Global variables" }
            if ($sizeLines) {
                $sizeLines | ForEach-Object { Write-Host "    $_" -ForegroundColor Yellow }
            }
            # Show last few lines of error
            $output | Select-Object -Last 5 | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkRed }
            $failCount++
            $results += @{ Name = $build.Name; Status = "Failed"; File = ""; Size = ""; MaxKB = 0; Percent = 0 }
        }
    } catch {
        Write-Host "  ✗ Error: $_" -ForegroundColor Red
        $failCount++
        $results += @{ Name = $build.Name; Status = "Error"; File = ""; Size = ""; MaxKB = 0; Percent = 0 }
    }
    
    Write-Host ""
}

# Cleanup temp directory
if (Test-Path $tempDir) {
    Remove-Item -Path $tempDir -Recurse -Force -ErrorAction SilentlyContinue
}

# Summary
Write-Host "========================================" -ForegroundColor Green
Write-Host " Build Summary" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host "  Successful: $successCount" -ForegroundColor $(if ($successCount -gt 0) { "Green" } else { "Gray" })
Write-Host "  Failed:     $failCount" -ForegroundColor $(if ($failCount -gt 0) { "Red" } else { "Gray" })
Write-Host ""

if ($successCount -gt 0) {
    Write-Host "Output files in: $releaseDir`n" -ForegroundColor Cyan
    
    # List full binaries (for USB flashing)
    Write-Host "  Full binaries (for USB flashing):" -ForegroundColor White
    Write-Host "  File                                              Size         Sketch / Max Bytes         Flash" -ForegroundColor White
    Write-Host "  ------------------------------------------------  -----------  -------------------------  ------" -ForegroundColor Gray
    foreach ($result in $results | Where-Object { $_.Status -eq "Success" }) {
        # Green: <75%, Yellow: 75-90%, DarkYellow: 90%+, Red: >=100%
        $usageColor = if ($result.Percent -ge 100) { "Red" }
                      elseif ($result.Percent -ge 90) { "DarkYellow" } 
                      elseif ($result.Percent -ge 75) { "Yellow" } 
                      else { "Green" }
        $fileName = $result.File.PadRight(48)
        $fileSizeStr = ("$($result.FileSizeKB) KB").PadRight(11)
        $sketchStr = ($result.SketchBytes.ToString("N0") + " / " + $result.MaxBytes.ToString("N0")).PadRight(25)
        $percentStr = ("$($result.Percent)" + "%").PadLeft(6)
        Write-Host "  $fileName  $fileSizeStr  $sketchStr  " -NoNewline -ForegroundColor White
        Write-Host $percentStr -ForegroundColor $usageColor
    }
    
    # List OTA binaries
    Write-Host ""
    Write-Host "  OTA binaries (for firmware updates via WebUI):" -ForegroundColor White
    Write-Host "  File                                              Size         Sketch / Max Bytes         Flash" -ForegroundColor White
    Write-Host "  ------------------------------------------------  -----------  -------------------------  ------" -ForegroundColor Gray
    foreach ($result in $results | Where-Object { $_.Status -eq "Success" }) {
        $otaFileName = $result.File -replace "_full\.bin$", "_ota.bin"
        $otaFilePath = Join-Path $releaseDir $otaFileName
        if (Test-Path $otaFilePath) {
            # Green: <75%, Yellow: 75-90%, DarkYellow: 90%+, Red: >=100%
            $usageColor = if ($result.Percent -ge 100) { "Red" }
                          elseif ($result.Percent -ge 90) { "DarkYellow" } 
                          elseif ($result.Percent -ge 75) { "Yellow" } 
                          else { "Green" }
            $otaFileSize = (Get-Item $otaFilePath).Length
            $otaFileSizeKB = [math]::Round($otaFileSize / 1024, 1)
            $otaFileNamePadded = $otaFileName.PadRight(48)
            $otaFileSizeStr = ("$otaFileSizeKB KB").PadRight(11)
            $sketchStr = ($result.SketchBytes.ToString("N0") + " / " + $result.MaxBytes.ToString("N0")).PadRight(25)
            $percentStr = ("$($result.Percent)" + "%").PadLeft(6)
            Write-Host "  $otaFileNamePadded  $otaFileSizeStr  $sketchStr  " -NoNewline -ForegroundColor White
            Write-Host $percentStr -ForegroundColor $usageColor
        }
    }
}

Write-Host ""

# Clean up build directories
$buildDir = Join-Path $PSScriptRoot "build"
if (Test-Path $tempDir) {
    Remove-Item -Path $tempDir -Recurse -Force -ErrorAction SilentlyContinue
}
if (Test-Path $buildDir) {
    Remove-Item -Path $buildDir -Recurse -Force -ErrorAction SilentlyContinue
}

if ($failCount -gt 0) {
    exit 1
}
