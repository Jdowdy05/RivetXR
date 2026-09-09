param(
    [string] $Metavr = 'metavr',
    [string] $ArtifactDir = $env:QUEST_NEWTON_ARTIFACT_DIR,
    [string] $FrankaDescriptionRoot = $env:FRANKA_DESCRIPTION_ROOT,
    [string] $MetaOpenXrSdkRoot = $env:META_OPENXR_SDK_ROOT,
    [string] $AndroidSdkRoot = $(if ($env:ANDROID_SDK_ROOT) { $env:ANDROID_SDK_ROOT } else { $env:ANDROID_HOME }),
    [string] $NdkRoot = $env:ANDROID_NDK_ROOT,
    [string] $JavaHome = $env:JAVA_HOME,
    [string] $HostPython = 'python',
    [string] $Ninja = 'ninja',
    [string] $BuildRoot = $env:QXR_BUILD_ROOT,
    [string] $Apk,
    [string] $Expected,
    [string] $OutputDir,
    [string] $Device,
    [switch] $SkipBuild,
    [ValidateRange(0, 3600)][int] $SoakSeconds = 0
)
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$package = 'com.questnewton'
. (Join-Path $PSScriptRoot 'task5_checks.ps1')

function Require-Path([string] $Path, [string] $Description) {
    if ([string]::IsNullOrWhiteSpace($Path) -or -not (Test-Path -LiteralPath $Path)) {
        throw "$Description is required: $Path"
    }
    return (Resolve-Path -LiteralPath $Path).Path
}

function Invoke-MetaChecked([string[]] $Arguments) {
    $result = (& $Metavr -d $selectedDevice @Arguments 2>&1 | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) { throw "Meta command failed ($($Arguments[0])): $result" }
    return $result
}

function Stop-AppChecked {
    Invoke-MetaChecked @('app', 'stop', $package) | Out-Null
    $processes = Invoke-MetaChecked @('shell', 'ps', '-A', '-w', '-o', 'CMD')
    if ($processes -notmatch '(?m)^\s*CMD\s*$') { throw 'Cannot verify process list after stop' }
    if ($processes -match ('(?m)^\s*' + [regex]::Escape($package) + '(?::[^\s]*)?\s*$')) {
        throw 'Application is still running after stop'
    }
}

function Assert-RunLog([string] $Log) {
    # This retains exactly the two narrowly scoped Horizon warning exceptions.
    # Ready here is intentionally ignored: startup messages may have rotated.
    Get-Task5Evidence $Log | Out-Null
    if ($Log -match '(?i)XR_ERROR_\w+|\bOXR\b[^\r\n]*(?:fail(?:ed|ure)?|error)') {
        throw "OpenXR failure in PID log: $($Matches[0])"
    }
}

function Invoke-RecordedRun([string] $Mode, [int] $Seconds, [string] $Name, [switch] $Capture) {
    $runId = [guid]::NewGuid().ToString('N')
    $prefix = Join-Path $evidenceRoot $Name
    $primaryError = $null
    $cleanupError = $null
    $attemptedLaunch = $false
    $result = $null
    try {
        Stop-AppChecked
        $attemptedLaunch = $true
        $launch = @('shell', 'am', 'start', '-W', '-n', "$package/.MainActivity", '--es', 'quest_newton_mode', $Mode,
            '--es', 'quest_newton_run_id', $runId)
        if ($Mode -eq 'soak') { $launch += @('--ei', 'quest_newton_soak_seconds', [string]$Seconds) }
        $launchText = Invoke-MetaChecked $launch
        Set-Content -LiteralPath "$prefix-launch.log" -Value $launchText -Encoding utf8
        if ($launchText -match '(?im)^\s*(Error|Exception):|Status:\s*(?!ok\b)\S+') { throw 'Activity launch did not succeed' }
        $pidText = Invoke-MetaChecked @('shell', 'pidof', $package)
        if ($pidText -notmatch '^\s*([1-9]\d*)\s*$') { throw 'Expected exactly one application PID' }
        $appProcessId = $Matches[1]
        $clock = [System.Diagnostics.Stopwatch]::StartNew()
        $timeout = if ($Mode -eq 'soak') { $Seconds + 60 } else { 60 }
        $ready = $false
        $smoke = $false
        $passthrough = $false
        $visuals = $false
        [uint64] $firstGeneration = 0
        [uint64] $lastGeneration = 0
        $firstPoll = $true
        $done = $false
        $nextProgress = 0
        do {
            $log = Invoke-MetaChecked @('adb', 'logcat', '--pid', $appProcessId, '--buffer', 'all', '--out-format', 'threadtime', '--lines', '0')
            Set-Content -LiteralPath "$prefix-latest.log" -Value $log -Encoding utf8
            if ($firstPoll) {
                Set-Content -LiteralPath "$prefix-first.log" -Value $log -Encoding utf8
                $firstPoll = $false
            }
            Assert-RunLog $log
            if ($log -match '\bQUEST_NEWTON_SMOKE_OK\b') { $smoke = $true }
            if ($log -match '\bQUEST_NEWTON_PASSTHROUGH_OK\b') { $passthrough = $true }
            foreach ($overlay in [regex]::Matches($log, 'QUEST_NEWTON_OVERLAY_OK generation=(\d+) bodies=12 visuals=11\b')) {
                $generation = [uint64]$overlay.Groups[1].Value
                if ($firstGeneration -eq 0) { $firstGeneration = $generation }
                $lastGeneration = [Math]::Max($lastGeneration, $generation)
                $visuals = $true
            }
            $readyPattern = "QUEST_NEWTON_READY mode=$Mode run_id=$runId refresh_hz=(\S+) visuals=11\b"
            $readyMarkers = [regex]::Matches($log, $readyPattern)
            if ($readyMarkers.Count -gt 1) { throw 'Duplicate READY markers' }
            if ($readyMarkers.Count -eq 1 -and -not $ready) {
                [double] $refresh = 0
                if (-not [double]::TryParse($readyMarkers[0].Groups[1].Value, [System.Globalization.NumberStyles]::Float,
                    [System.Globalization.CultureInfo]::InvariantCulture, [ref]$refresh) -or
                    -not [double]::IsFinite($refresh) -or [Math]::Abs($refresh - 90) -gt .1) { throw 'READY refresh must be 90 Hz' }
                Set-Content -LiteralPath "$prefix-ready.log" -Value $log -Encoding utf8
                $ready = $true
            }
            $donePattern = if ($Mode -eq 'trace') { "QUEST_NEWTON_TRACE_DONE run_id=$runId states=1000\b" } else { "QUEST_NEWTON_SOAK_DONE run_id=$runId\b" }
            $done = $log -match $donePattern
            if ((Invoke-MetaChecked @('shell', 'pidof', $package)).Trim() -ne $appProcessId) { throw 'Application PID changed during replay' }
            if ($done) { break }
            if ($clock.Elapsed.TotalSeconds -ge $nextProgress) {
                Write-Host "$Name elapsed=$([int]$clock.Elapsed.TotalSeconds)s ready=$ready; waiting for nonce-bound completion"
                $nextProgress = $clock.Elapsed.TotalSeconds + 10
            }
            Start-Sleep -Milliseconds $(if ($Mode -eq 'soak') { 2000 } else { 250 })
        } while ($clock.Elapsed.TotalSeconds -lt $timeout)
        if (-not ($done -and $ready -and $smoke -and $passthrough -and $visuals -and $lastGeneration -gt $firstGeneration)) {
            throw "Recorded $Mode evidence incomplete: done=$done ready=$ready smoke=$smoke passthrough=$passthrough visuals=$visuals generations=$firstGeneration..$lastGeneration"
        }
        $privateFile = if ($Mode -eq 'trace') { 'newton_trace.jsonl' } else { 'newton_metrics.json' }
        $destination = "$prefix-$privateFile"
        Invoke-MetaChecked @('shell', 'run-as', $package, 'cat', "files/$privateFile") |
            Set-Content -LiteralPath $destination -Encoding utf8
        if ($Mode -eq 'soak') {
            Invoke-MetaChecked @('shell', 'run-as', $package, 'cat', 'files/newton_soak_progress.jsonl') |
                Set-Content -LiteralPath "$prefix-progress.jsonl" -Encoding utf8
        }
        if ($Capture) {
            $capturePath = Join-Path $evidenceRoot 'task8-compositor.png'
            Invoke-MetaChecked @('capture', 'screenshot', '--method', 'metacam', '--width', '1536', '--height', '1536', '--output', $capturePath) | Out-Null
            if (-not (Test-Path -LiteralPath $capturePath -PathType Leaf) -or (Get-Item -LiteralPath $capturePath).Length -eq 0) { throw 'Compositor capture missing' }
            $log = Invoke-MetaChecked @('adb', 'logcat', '--pid', $appProcessId, '--buffer', 'all', '--out-format', 'threadtime', '--lines', '0')
            Set-Content -LiteralPath "$prefix-latest.log" -Value $log -Encoding utf8
            Assert-RunLog $log
            if ((Invoke-MetaChecked @('shell', 'pidof', $package)).Trim() -ne $appProcessId) { throw 'Application PID changed during capture' }
        }
        $result = [pscustomobject]@{ RunId = $runId; File = $destination; ElapsedSeconds = $clock.Elapsed.TotalSeconds }
    } catch { $primaryError = $_ }
    finally {
        if ($attemptedLaunch) {
            try { Stop-AppChecked } catch { $cleanupError = $_ }
        }
    }
    if ($cleanupError) {
        $original = if ($primaryError) { $primaryError.Exception.Message } else { 'none' }
        throw "Cleanup failed: $($cleanupError.Exception.Message); original failure: $original"
    }
    if ($primaryError) { throw $primaryError }
    return $result
}

if (-not $Expected) { $Expected = Join-Path $PSScriptRoot 'expected_trace.json' }
if (-not $Apk) { $Apk = Join-Path $repoRoot 'quest/app/build/outputs/apk/debug/quest-newton-debug.apk' }
if (-not $OutputDir) { $OutputDir = Join-Path $repoRoot ('out/device-task8/' + [guid]::NewGuid().ToString('N')) }
$Expected = Require-Path $Expected 'expected trace'
$artifactRoot = Require-Path $ArtifactDir 'artifact directory'
$sdkRoot = Require-Path $AndroidSdkRoot 'Android SDK'
$ndk = Require-Path $NdkRoot 'Android NDK'
$java = Require-Path $JavaHome 'JDK 17'
$manifestPath = Require-Path (Join-Path $artifactRoot 'artifact_manifest.json') 'physics manifest'
$manifestHash = (Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
$expectedDocument = Get-Content -Raw -LiteralPath $Expected | ConvertFrom-Json
if ($expectedDocument.physics_manifest_sha256 -ne $manifestHash) { throw 'Expected trace belongs to a different physics bundle' }
$evidenceRoot = [System.IO.Path]::GetFullPath($OutputDir)
if (Test-Path -LiteralPath $evidenceRoot) { throw 'OutputDir must be fresh and nonexistent' }
& git -C $repoRoot check-ignore --quiet -- (Join-Path $evidenceRoot 'evidence.json')
if ($LASTEXITCODE -ne 0) { throw 'OutputDir must be inside an ignored repository output directory' }
New-Item -ItemType Directory -Path $evidenceRoot | Out-Null

if (-not $SkipBuild) {
    $buildArguments = @{ ArtifactDir = $artifactRoot; MetaOpenXrSdkRoot = $MetaOpenXrSdkRoot; AndroidSdkRoot = $sdkRoot;
        NdkRoot = $ndk; JavaHome = $java; FrankaDescriptionRoot = $FrankaDescriptionRoot; HostPython = $HostPython; Ninja = $Ninja }
    if ($BuildRoot) { $buildArguments.BuildRoot = $BuildRoot }
    & (Join-Path $repoRoot 'tools/build_quest.ps1') @buildArguments
    if (-not $?) { throw 'APK build failed' }
}
$apkPath = Require-Path $Apk 'APK'
$aapt2 = Get-ChildItem -LiteralPath (Join-Path $sdkRoot 'build-tools') -Directory | Sort-Object Name -Descending |
    ForEach-Object { Join-Path $_.FullName 'aapt2.exe' } | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
$apksigner = Get-ChildItem -LiteralPath (Join-Path $sdkRoot 'build-tools') -Directory | Sort-Object Name -Descending |
    ForEach-Object { Join-Path $_.FullName 'apksigner.bat' } | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
$readelf = Require-Path (Join-Path $ndk 'toolchains/llvm/prebuilt/windows-x86_64/bin/llvm-readelf.exe') 'llvm-readelf'
$aapt2 = Require-Path $aapt2 'aapt2'
$apksigner = Require-Path $apksigner 'apksigner'
$oldJava = $env:JAVA_HOME
try {
    $env:JAVA_HOME = $java
    $javaVersion = (& (Join-Path $java 'bin/java.exe') -version 2>&1 | Out-String)
    if ($LASTEXITCODE -ne 0 -or $javaVersion -notmatch 'version "17\.') { throw 'JDK 17 is required' }
    & pwsh -NoProfile -File (Join-Path $repoRoot 'tests/android_apk_contract.ps1') -Apk $apkPath -Aapt2Path $aapt2 -ReadElfPath $readelf -ApkSignerPath $apksigner
    if ($LASTEXITCODE -ne 0) { throw 'APK contract verification failed' }
} finally { $env:JAVA_HOME = $oldJava }

# Bind the verified APK's exact assets to this expected reference and bundle.
$archive = [System.IO.Compression.ZipFile]::OpenRead($apkPath)
try {
    foreach ($pin in @(
        @{ Name = 'assets/newton/artifact_manifest.json'; Hash = $manifestHash },
        @{ Name = 'assets/verification/controller_trace.json'; Hash = $expectedDocument.trace_sha256 }
    )) {
        $entries = @($archive.Entries | Where-Object { $_.FullName -eq $pin.Name })
        if ($entries.Count -ne 1) { throw "APK must contain exactly one $($pin.Name)" }
        $stream = $entries[0].Open()
        $hasher = [System.Security.Cryptography.SHA256]::Create()
        try { $digest = [Convert]::ToHexString($hasher.ComputeHash($stream)).ToLowerInvariant() }
        finally { $stream.Dispose(); $hasher.Dispose() }
        if ($digest -ne $pin.Hash) { throw "APK reference hash mismatch: $($pin.Name)" }
    }
} finally { $archive.Dispose() }

# First DEVICE command, after local build/contract checks. Never save identifiers.
$deviceText = (& $Metavr device list --format json 2>&1 | Out-String)
if ($LASTEXITCODE -ne 0) { throw 'Meta device discovery failed' }
$devices = @($deviceText | ConvertFrom-Json | Where-Object {
    $_.state -eq 'device' -and $_.model -eq 'Quest 3S' -and (-not $Device -or $_.id -eq $Device)
})
if ($devices.Count -ne 1) { throw 'Select exactly one currently authorized Quest 3S using -Device' }
$selectedDevice = $devices[0].id
Invoke-MetaChecked @('app', 'install', '--replace', $apkPath) | Out-Null
$packageInfo = Invoke-MetaChecked @('shell', 'dumpsys', 'package', $package)
if ($packageInfo -notmatch 'Package \[com\.questnewton\]' -or $packageInfo -notmatch '\bversionCode=1\s' -or
    $packageInfo -notmatch '(?m)^\s*versionName=1\.0\s*$') { throw 'Installed package must be com.questnewton versionCode 1, versionName 1.0' }
# Save only confirmed fields, avoiding device/account fields in dumpsys output.
$metadata = [ordered]@{ Package = $package; VersionCode = 1; VersionName = '1.0'; DeviceModel = 'Quest 3S';
    ApkSha256 = (Get-FileHash -LiteralPath $apkPath -Algorithm SHA256).Hash.ToLowerInvariant();
    PhysicsManifestSha256 = $manifestHash; TraceSha256 = $expectedDocument.trace_sha256;
    StartedUtc = [DateTime]::UtcNow.ToString('o'); PerformanceComplete = $false }
$metadata | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $evidenceRoot 'metadata.json') -Encoding utf8
$runA = Invoke-RecordedRun -Mode trace -Seconds 0 -Name 'run-a'
$runB = Invoke-RecordedRun -Mode trace -Seconds 0 -Name 'run-b' -Capture
Push-Location $repoRoot
try {
    & $HostPython -m tools.verification.verify_device_trace --expected $Expected --trace-a $runA.File --trace-b $runB.File `
        --run-id-a $runA.RunId --run-id-b $runB.RunId --output (Join-Path $evidenceRoot 'functional-report.json')
    if ($LASTEXITCODE -ne 0) { throw 'Functional trace validation failed' }
    if ($SoakSeconds -gt 0) {
        $soak = Invoke-RecordedRun -Mode soak -Seconds $SoakSeconds -Name 'soak'
        & $HostPython -m tools.verification.verify_device_trace --expected $Expected --metrics $soak.File --run-id $soak.RunId `
            --minimum-soak-seconds $SoakSeconds --output (Join-Path $evidenceRoot 'soak-report.json')
        if ($LASTEXITCODE -ne 0) { throw 'Native soak evidence validation failed' }
    }
} finally { Pop-Location }
Write-Output "Functional recorded replay passed; app stopped. Evidence: $evidenceRoot"
if ($SoakSeconds -gt 0) { Write-Output "Native $SoakSeconds-second soak evidence passed; app stopped." }
Write-Output 'External Perfetto, thermal/frequency/frame analysis and live Touch/visual acceptance remain separate gates.'
