# Pure evidence checks shared by the device runner and host fixtures.
function Assert-Task5Controllers([object[]] $Controllers) {
    foreach ($hand in @('LeftHand', 'RightHand')) {
        $records = @($Controllers | Where-Object { $_.type -eq $hand })
        if ($records.Count -ne 1 -or $records[0].status -ne 'CONNECTED_ACTIVE' -or
            $records[0].trackingStatus -notmatch '\bPOSITION\b') {
            throw "Wake both controllers: $hand must be a distinct connected-active record with POSITION tracking"
        }
    }
}

function Get-Task5Evidence([string] $Log) {
    # Horizon runtime v86 emits these two warning-level provider probes even
    # while passthrough and eye rendering succeed. Keep them in the saved log
    # and report their count. All other missing symbols, including E/F severity
    # from the same providers, still fail the gate.
    $runtimeWarningPattern = '(?m)^\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}\.\d{3}\s+\d+\s+\d+\s+W\s+(?:OrthofitClientProvider:\s+OrthofitClientProvider: Failed to get orthofit client functions: undefined symbol: destroyOrthofitClient|ImageSuitabilityDetectionClientProvider:\s+ImageSuitabilityDetectionClientProvider: Failed to get image suitability detection client functions: undefined symbol: destroyImageSuitabilityDetectionClient)\r?$'
    $runtimeWarningCount = [regex]::Matches($Log, $runtimeWarningPattern).Count
    $checkedLog = [regex]::Replace($Log, $runtimeWarningPattern, '')
    $badPattern = '(?i)FATAL EXCEPTION|Fatal signal|SIG(SEGV|ABRT)|dlopen failed|UnsatisfiedLinkError|undefined symbol|non[- ]finite|QUEST_NEWTON_\w*FAIL|GL_INVALID_\w+|GL_OUT_OF_MEMORY|GL error on line|EGL_BAD_\w+'
    if ($checkedLog -match $badPattern) { throw "PID-scoped log contains failure marker: $($Matches[0])" }
    $smokeCount = [regex]::Matches($Log, '\bQUEST_NEWTON_SMOKE_OK\b').Count
    if ($smokeCount -gt 1) { throw 'Duplicate smoke success markers in the launched PID' }
    if ($smokeCount -eq 1) {
        $smoke = [regex]::Match($Log, 'QUEST_NEWTON_SMOKE_OK first=(\S+) last=(\S+)')
        if (-not $smoke.Success) { throw 'Smoke marker is missing joint state' }
        foreach ($field in 1, 2) {
            [double] $value = 0
            if (-not [double]::TryParse($smoke.Groups[$field].Value, [System.Globalization.NumberStyles]::Float,
                [System.Globalization.CultureInfo]::InvariantCulture, [ref]$value) -or -not [double]::IsFinite($value)) {
                throw 'Smoke joint state must be finite numeric values'
            }
        }
    }
    $overlays = [regex]::Matches($Log, 'QUEST_NEWTON_OVERLAY_OK generation=(\d+) bodies=(\d+)')
    [uint64] $first = 0
    [uint64] $last = 0
    foreach ($overlay in $overlays) {
        [uint64] $generation = $overlay.Groups[1].Value
        if ($overlay.Groups[2].Value -ne '12' -or $generation -eq 0 -or $generation -lt $last) {
            throw 'Overlay must contain twelve bodies and monotonically advancing valid generations'
        }
        if ($first -eq 0) { $first = $generation }
        $last = $generation
    }
    return [pscustomobject]@{
        Ready = $smokeCount -eq 1 -and $Log -match '\bQUEST_NEWTON_PASSTHROUGH_OK\b' -and $last -gt $first
        FirstGeneration = $first
        LastGeneration = $last
        RuntimeWarningCount = $runtimeWarningCount
    }
}
