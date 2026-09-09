$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'task5_checks.ps1')

function Rejects([scriptblock] $Operation) {
    $rejected = $false
    try { & $Operation | Out-Null } catch { $rejected = $true }
    if (-not $rejected) { throw 'Invalid device evidence was accepted' }
}

$controllers = @(
    [pscustomobject]@{ type='LeftHand'; status='CONNECTED_ACTIVE'; trackingStatus='POSITION' },
    [pscustomobject]@{ type='RightHand'; status='CONNECTED_ACTIVE'; trackingStatus='POSITION' }
)
Assert-Task5Controllers $controllers
Rejects { Assert-Task5Controllers @($controllers[0], $controllers[0]) }
Rejects { Assert-Task5Controllers @($controllers[1]) }
$controllers[0].status = 'CONNECTED_INACTIVE'
Rejects { Assert-Task5Controllers $controllers }
$controllers[0].status = 'CONNECTED_ACTIVE'
$controllers[1].trackingStatus = 'NONE'
Rejects { Assert-Task5Controllers $controllers }

$validLog = @'
I QuestNewton: QUEST_NEWTON_PASSTHROUGH_OK
I QuestNewton: QUEST_NEWTON_SMOKE_OK first=0.000000 last=0.000000
I QuestNewton: QUEST_NEWTON_OVERLAY_OK generation=1 bodies=12
I QuestNewton: QUEST_NEWTON_OVERLAY_OK generation=101 bodies=12
'@
$evidence = Get-Task5Evidence $validLog
if (-not $evidence.Ready -or $evidence.FirstGeneration -ne 1 -or $evidence.LastGeneration -ne 101) {
    throw 'Complete advancing overlay evidence was not accepted'
}
foreach ($incomplete in @('', ($validLog -replace 'QUEST_NEWTON_PASSTHROUGH_OK', ''),
    ($validLog -replace 'QUEST_NEWTON_SMOKE_OK', ''),
    ($validLog -replace 'generation=101', 'generation=1'))) {
    if ((Get-Task5Evidence $incomplete).Ready) { throw 'Incomplete/stale evidence was accepted' }
}
Rejects { Get-Task5Evidence ($validLog + "`nQUEST_NEWTON_SMOKE_OK first=0 last=0") }
Rejects { Get-Task5Evidence ($validLog -replace 'generation=101', 'generation=0') }
Rejects { Get-Task5Evidence ($validLog -replace 'bodies=12', 'bodies=11') }
Rejects { Get-Task5Evidence ($validLog -replace 'first=0.000000', 'first=NaN') }
foreach ($failure in @('QUEST_NEWTON_SMOKE_FAIL stage=step', 'Fatal signal 11',
    'undefined symbol', 'non-finite state', 'GL_INVALID_OPERATION', 'EGL_BAD_CONTEXT',
    'GL error on line 103: 0x506')) {
    Rejects { Get-Task5Evidence ($validLog + "`n" + $failure) }
}
$runtimeWarnings = @'
09-04 19:13:47.747 123 456 W OrthofitClientProvider: OrthofitClientProvider: Failed to get orthofit client functions: undefined symbol: destroyOrthofitClient
09-04 19:13:47.754 123 456 W ImageSuitabilityDetectionClientProvider: ImageSuitabilityDetectionClientProvider: Failed to get image suitability detection client functions: undefined symbol: destroyImageSuitabilityDetectionClient
'@
$withWarnings = Get-Task5Evidence ($validLog + "`n" + $runtimeWarnings)
if (-not $withWarnings.Ready -or $withWarnings.RuntimeWarningCount -ne 2) {
    throw 'Observed runtime warnings must be reported without rejecting advancing overlay evidence'
}
Rejects { Get-Task5Evidence ($validLog + "`n" + ($runtimeWarnings -replace ' W ', ' E ')) }
Rejects { Get-Task5Evidence ($validLog + "`n" + ($runtimeWarnings -replace 'destroyOrthofitClient', 'requiredAppSymbol')) }
Write-Output 'Task 5 controller and PID-log evidence checks passed'
