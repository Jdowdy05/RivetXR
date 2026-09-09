param(
    [Parameter(Mandatory)][string] $Metavr,
    [Parameter(Mandatory)][string] $Apk,
    [Parameter(Mandatory)][string] $OutputDir,
    [string] $Device,
    [ValidateRange(5, 60)][int] $TimeoutSeconds = 30
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'task5_checks.ps1')
$package = 'com.questnewton'

# Discovery is always the first device command. Never persist device identifiers.
$deviceText = (& $Metavr device list --format json 2>&1 | Out-String)
if ($LASTEXITCODE -ne 0) { throw 'Meta device discovery failed' }
$devices = @($deviceText | ConvertFrom-Json | Where-Object {
    $_.state -eq 'device' -and $_.model -eq 'Quest 3S' -and (-not $Device -or $_.id -eq $Device)
})
if ($devices.Count -ne 1) { throw 'Select exactly one authorized Quest 3S using -Device' }
$selectedDevice = $devices[0].id
if (-not (Test-Path -LiteralPath $Apk -PathType Leaf)) { throw 'Build the strict APK before running this check' }
$apkPath = (Resolve-Path -LiteralPath $Apk).Path
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$evidenceRoot = (Resolve-Path -LiteralPath $OutputDir).Path
$logPath = Join-Path $evidenceRoot 'task5-pid.log'
$capturePath = Join-Path $evidenceRoot 'task5-compositor.png'
if (Test-Path -LiteralPath $capturePath) { throw 'Use a fresh OutputDir so a stale screenshot cannot pass' }
$primaryError = $null
$cleanupError = $null
$launched = $false

function Invoke-MetaChecked([string[]] $Arguments) {
    $result = (& $Metavr -d $selectedDevice @Arguments 2>&1 | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) { throw "Meta command failed ($($Arguments[0])): $result" }
    return $result
}

try {
    $controllerDocument = (Invoke-MetaChecked @('device', 'controllers', '--format', 'json')) | ConvertFrom-Json
    $controllers = if ($controllerDocument -is [array]) { @($controllerDocument) } else { @($controllerDocument.controllers) }
    Assert-Task5Controllers $controllers
    Invoke-MetaChecked @('app', 'install', '--replace', $apkPath) | Out-Null
    # Set before launch so cleanup also covers a launch that partially succeeds.
    $launched = $true
    Invoke-MetaChecked @('app', 'launch', '--cold-start', '--wait-for-idle', '--wait-timeout', '15', $package) | Out-Null
    $pidText = Invoke-MetaChecked @('shell', 'pidof', $package)
    if ($pidText -notmatch '^\s*([1-9]\d*)\s*$') { throw 'Expected exactly one application PID' }
    $appProcessId = $Matches[1]
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    do {
        $log = Invoke-MetaChecked @('adb', 'logcat', '--pid', $appProcessId, '--buffer', 'all', '--out-format', 'threadtime', '--lines', '0')
        Set-Content -LiteralPath $logPath -Value $log
        $evidence = Get-Task5Evidence $log
        if ($evidence.Ready) { break }
        Start-Sleep -Milliseconds 250
    } while ((Get-Date) -lt $deadline)
    if (-not $evidence.Ready) { throw 'Timed out waiting for passthrough, one smoke success, and advancing twelve-body rendered snapshots' }
    Invoke-MetaChecked @('capture', 'screenshot', '--method', 'metacam', '--width', '1536', '--height', '1536', '--output', $capturePath) | Out-Null
    if (-not (Test-Path -LiteralPath $capturePath -PathType Leaf) -or (Get-Item -LiteralPath $capturePath).Length -eq 0) {
        throw 'Compositor screenshot was not created'
    }
    if ((Invoke-MetaChecked @('shell', 'pidof', $package)).Trim() -ne $appProcessId) { throw 'Application PID changed during capture' }
    $log = Invoke-MetaChecked @('adb', 'logcat', '--pid', $appProcessId, '--buffer', 'all', '--out-format', 'threadtime', '--lines', '0')
    Set-Content -LiteralPath $logPath -Value $log
    $evidence = Get-Task5Evidence $log
    if (-not $evidence.Ready) { throw 'Final PID-log evidence is incomplete' }
    [pscustomobject]@{
        ApkSha256 = (Get-FileHash -LiteralPath $apkPath -Algorithm SHA256).Hash.ToLowerInvariant()
        FirstGeneration = $evidence.FirstGeneration
        LastGeneration = $evidence.LastGeneration
        RuntimeWarningCount = $evidence.RuntimeWarningCount
        Screenshot = $capturePath
        VisualInspection = 'required; automated evidence is not visual acceptance'
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $evidenceRoot 'task5-evidence.json')
} catch {
    $primaryError = $_
} finally {
    if ($launched) {
        try {
            Invoke-MetaChecked @('app', 'stop', $package) | Out-Null
            # A successful process-list query distinguishes absence from a
            # disconnected device. pidof alone uses a nonzero exit for both.
            $processes = Invoke-MetaChecked @('shell', 'ps', '-A', '-w', '-o', 'CMD')
            if ($processes -notmatch '(?m)^\s*CMD\s*$') { throw 'Could not verify the process list after stop' }
            if ($processes -match ('(?m)^\s*' + [regex]::Escape($package) + '(?::[^\s]*)?\s*$')) {
                throw 'Application is still running after stop'
            }
        } catch { $cleanupError = $_ }
    }
}
if ($cleanupError) { throw "Cleanup failed: $($cleanupError.Exception.Message); original result: $($primaryError.Exception.Message)" }
if ($primaryError) { throw $primaryError }
Write-Output "Task 5 automated device evidence passed; app stopped. Inspect $capturePath before declaring visual acceptance."
