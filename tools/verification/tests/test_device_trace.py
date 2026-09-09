"""Synthetic parser fixtures only; these are never production reference physics."""
import copy
import importlib.util
import json
import math
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


def engaged(index):
    return any(a <= index <= b for a, b in ((30, 349), (380, 399), (440, 599), (640, 849), (900, 979)))


def timing(count=10000):
    return dict(available=True, count=count, mean_us=100., p95_us=150., p99_us=200., max_us=300., over_budget_count=0)


def policy():
    return dict(name='newton_soft_stop_state_v1', arm_rad=1e-5, finger_m=1e-6, target_limits='strict')


def state_statistics(count=10000):
    return dict(samples_checked=count, strict_excursion_samples=0, per_joint_excursion_counts=[0] * 9,
                max_excursions=[0.] * 9, allowance_violations=0)


def fixture():
    expected = dict(schema_version=1, trace_sha256='a' * 64, physics_manifest_sha256='b' * 64,
                    tolerances=dict(joint_q=1e-5, joint_qd=1e-4), samples=[],
                    physics_substeps=10000, substeps_per_sample=10, timestep_seconds=.001,
                    state_limit_policy=policy(), state_limits=state_statistics())
    target = [0., -.569, 0., -2.81, 0., 3.037, .741]
    states = []
    for index in range(1000):
        if engaged(index):
            target = [.03 * math.sin(index / 100), -.569, 0., -2.81, 0., 3.037, .741]
        row = dict(index=index, engaged=engaged(index), calibrated=index >= 1,
                   targets=target[:], q=target[:] + [.03999999910593033] * 2, qd=[0.] * 9)
        expected['samples'].append(row)
        states.append(dict(type='state', sim_time_s=.01 * (index + 1), body_q=[0., 0., 0., 0., 0., 0., 1.] * 12, **copy.deepcopy(row)))
    header = dict(type='header', schema_version=1, mode='trace', run_id='run_a',
                  trace_sha256=expected['trace_sha256'], physics_manifest_sha256=expected['physics_manifest_sha256'],
                  substeps_per_sample=10, state_count=1000, state_limit_policy=policy())
    a = [header, *states, dict(type='complete', state_count=1000, physics=timing(), state_limits=state_statistics())]
    b = copy.deepcopy(a)
    b[0]['run_id'] = 'run_b'
    return expected, a, b


class DeviceTraceTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('tools.verification.verify_device_trace'), 'device validator must exist')
        from tools.verification import verify_device_trace
        self.validator = verify_device_trace
        self.expected, self.a, self.b = fixture()

    def verify(self):
        return self.validator.verify_runs(self.expected, self.a, self.b, 'run_a', 'run_b')

    def test_valid_replays_do_not_claim_full_performance_evidence(self):
        result = self.verify()
        self.assertTrue(result['functional_pass'])
        self.assertFalse(result['performance_complete'])

    def test_rejects_corrupted_contract(self):
        changes = {
            'hash': lambda a: a[0].update(trace_sha256='c' * 64),
            'nonce': lambda a: a[0].update(run_id='old'),
            'NaN': lambda a: a[80]['q'].__setitem__(0, float('nan')),
            'index': lambda a: a[80].update(index=78),
            'count': lambda a: a.pop(80),
            'time': lambda a: a[80].update(sim_time_s=99),
            'finger limit': lambda a: a[80]['q'].__setitem__(8, .041),
            'arm limit': lambda a: a[80]['q'].__setitem__(3, 0),
            'engagement': lambda a: a[412].update(engaged=True),
            'calibration': lambda a: a[80].update(calibrated=False),
            'body quaternion': lambda a: a[80]['body_q'].__setitem__(6, .8),
            'timing order': lambda a: a[-1]['physics'].update(p95_us=250),
            'timing threshold': lambda a: a[-1]['physics'].update(p99_us=2000, max_us=2000),
            'physics count': lambda a: a[-1]['physics'].update(count=9999),
            'unavailable': lambda a: a[-1]['physics'].update(available=False),
            'nonbool': lambda a: a[80].update(engaged=1),
            'empty': lambda a: a.clear(),
        }
        for name, mutate in changes.items():
            with self.subTest(name=name):
                candidate = copy.deepcopy(self.a)
                mutate(candidate)
                with self.assertRaises(ValueError):
                    self.validator.verify_runs(self.expected, candidate, self.b, 'run_a', 'run_b')

    def test_rejects_identical_ids(self):
        with self.assertRaises(ValueError):
            self.validator.verify_runs(self.expected, self.a, self.a, 'run_a', 'run_a')

    def test_independent_phase_gate_rejects_corrupted_golden(self):
        for rows in (self.expected['samples'], self.a[1:-1], self.b[1:-1]):
            rows[410]['engaged'] = True
        with self.assertRaisesRegex(ValueError, 'engaged'):
            self.verify()

    def test_inactive_targets_must_freeze_even_if_golden_changed(self):
        for rows in (self.expected['samples'], self.a[1:-1], self.b[1:-1]):
            rows[350]['targets'][0] += .02
        with self.assertRaisesRegex(ValueError, 'target'):
            self.verify()

    def test_static_golden_and_device_cannot_pass(self):
        for rows in (self.expected['samples'], self.a[1:-1], self.b[1:-1]):
            for row in rows:
                row['q'] = self.expected['samples'][0]['q'][:]
                row['targets'] = self.expected['samples'][0]['targets'][:]
        with self.assertRaisesRegex(ValueError, 'motion'):
            self.verify()

    def test_absolute_tolerances_and_separate_android_repeatability(self):
        self.a[80]['q'][0] += 9e-6
        self.b[80]['q'][0] -= 9e-6
        with self.assertRaisesRegex(ValueError, 'Android'):
            self.verify()
        self.a[80]['q'][0] += 9e-6
        with self.assertRaisesRegex(ValueError, 'expected'):
            self.verify()

    def test_declared_tolerances_cannot_be_relaxed(self):
        self.expected['tolerances']['joint_q'] = .01
        with self.assertRaises(ValueError):
            self.verify()

    def test_state_policy_must_match_approved_allowances_everywhere(self):
        for location in ('expected', 'header', 'metrics'):
            for key, value in (('arm_rad', 1e-4), ('finger_m', 1e-5), ('name', 'other'), ('target_limits', 'tolerant')):
                with self.subTest(location=location, key=key):
                    expected, a, b = fixture()
                    metrics = self.metrics()
                    target = {'expected': expected, 'header': a[0], 'metrics': metrics}[location]
                    target['state_limit_policy'][key] = value
                    with self.assertRaisesRegex(ValueError, 'policy'):
                        if location == 'metrics':
                            self.validator.verify_metrics(expected, metrics, 'soak_1')
                        else:
                            self.validator.verify_runs(expected, a, b, 'run_a', 'run_b')

    def test_raw_state_excursions_are_accepted_and_must_be_reported(self):
        for rows in (self.expected['samples'], self.a[1:-1], self.b[1:-1]):
            rows[79]['q'][7] = .03999999910593033 + 5e-7
        for document in (self.expected, self.a[-1], self.b[-1]):
            document['state_limits'].update(strict_excursion_samples=1,
                per_joint_excursion_counts=[0] * 7 + [1, 0], max_excursions=[0.] * 7 + [5.000000000005e-7, 0.])
        self.assertTrue(self.verify()['functional_pass'])
        self.a[-1]['state_limits'] = state_statistics()
        with self.assertRaisesRegex(ValueError, 'excursion'):
            self.verify()

    def test_statistics_reject_missing_inconsistent_or_unchecked_substeps(self):
        changes = [lambda s: s.update(samples_checked=9999), lambda s: s.update(allowance_violations=1),
                   lambda s: s.update(strict_excursion_samples=10001),
                   lambda s: s['per_joint_excursion_counts'].__setitem__(7, 1),
                   lambda s: s['max_excursions'].__setitem__(7, 1e-7),
                   lambda s: s.update(strict_excursion_samples=True),
                   lambda s: s['max_excursions'].__setitem__(7, float('nan')),
                   lambda s: s.update(strict_excursion_samples=1, per_joint_excursion_counts=[0]*7+[1,0],
                                      max_excursions=[0.]*7+[1.001e-6,0.])]
        for mutate in changes:
            with self.subTest(mutation=mutate):
                candidate = copy.deepcopy(self.a)
                mutate(candidate[-1]['state_limits'])
                with self.assertRaises(ValueError):
                    self.validator.verify_runs(self.expected, candidate, self.b, 'run_a', 'run_b')
        del self.a[-1]['state_limits']
        with self.assertRaises(ValueError):
            self.verify()

    def test_targets_remain_strict_at_float32_manifest_boundary(self):
        self.a[80]['targets'][0] = 2.8973000049591064 + 1e-8
        with self.assertRaisesRegex(ValueError, 'targets joint limit'):
            self.verify()

    def test_joint_velocity_comparison_tolerance_is_unchanged(self):
        self.a[80]['qd'][0] = 9.99e-5
        self.assertTrue(self.verify()['functional_pass'])
        self.a[80]['qd'][0] = 1.001e-4
        with self.assertRaisesRegex(ValueError, 'qd differs'):
            self.verify()

    def metrics(self):
        return dict(schema_version=1, mode='soak', run_id='soak_1', trace_sha256='a' * 64,
                    physics_manifest_sha256='b' * 64, completed=True, duration_seconds=1800.1,
                    soak_seconds_requested=1800, valid_state_count=180000,
                    state_limit_policy=policy(), state_limits=state_statistics(1800000),
                    render_frames=162000, refresh_hz=90., gpu_timer_supported=True, gpu_disjoint_count=0,
                    physics=timing(1800000), render_cpu=timing(162000), render_gpu=timing(162000))

    def test_soak_is_native_evidence_only_and_rejects_missing_coverage(self):
        valid = self.metrics()
        report = self.validator.verify_metrics(self.expected, valid, 'soak_1', 1800)
        self.assertTrue(report['native_soak_evidence'])
        self.assertFalse(report['performance_complete'])
        for key, value in [('completed', False), ('duration_seconds', 1799), ('run_id', 'old'),
                           ('render_frames', 1), ('valid_state_count', 0), ('state_limits', state_statistics(999)),
                           ('refresh_hz', 72), ('duration_seconds', float('inf'))]:
            with self.subTest(key=key):
                bad = copy.deepcopy(valid)
                bad[key] = value
                with self.assertRaises(ValueError):
                    self.validator.verify_metrics(self.expected, bad, 'soak_1', 1800)

    def test_soak_timing_counts_must_match_native_work(self):
        for field, increment in (('physics', 1), ('render_cpu', 1), ('render_gpu', 9 * 162000)):
            with self.subTest(field=field):
                bad = self.metrics()
                bad[field]['count'] += increment
                with self.assertRaisesRegex(ValueError, 'count'):
                    self.validator.verify_metrics(self.expected, bad, 'soak_1', 1800)

    def test_soak_allows_unsupported_gpu_and_disjoint_query_loss(self):
        metrics = self.metrics()
        metrics['render_gpu']['count'] = 1234
        metrics['gpu_disjoint_count'] = 2
        self.assertTrue(self.validator.verify_metrics(self.expected, metrics, 'soak_1', 1800)['native_soak_evidence'])
        metrics['gpu_timer_supported'] = False
        metrics['render_gpu'] = dict(available=False, count=0, mean_us=0., p95_us=0., p99_us=0., max_us=0., over_budget_count=0)
        self.assertTrue(self.validator.verify_metrics(self.expected, metrics, 'soak_1', 1800)['native_soak_evidence'])

    def test_cli_writes_report_and_nonzero_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'expected.json').write_text(json.dumps(self.expected))
            for name, rows in [('a', self.a), ('b', self.b)]:
                (root / f'{name}.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
            command = [sys.executable, '-m', 'tools.verification.verify_device_trace', '--expected', str(root / 'expected.json'),
                       '--trace-a', str(root / 'a.jsonl'), '--trace-b', str(root / 'b.jsonl'),
                       '--run-id-a', 'run_a', '--run-id-b', 'run_b', '--output', str(root / 'report.json')]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(json.loads((root / 'report.json').read_text())['functional_pass'])
            (root / 'a.jsonl').write_text('NaN\n')
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(json.loads((root / 'report.json').read_text())['functional_pass'])

    def test_json_reader_rejects_duplicate_keys_and_nonfinite_values(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'bad.json'
            for raw in ('{"index":0,"index":1}', '{"value":NaN}', '{"value":Infinity}'):
                source.write_text(raw)
                with self.assertRaises(ValueError):
                    self.validator.load_json(source)


@unittest.skipUnless(shutil.which('pwsh'), 'PowerShell runner tests require pwsh')
class DeviceRunnerTests(unittest.TestCase):
    def test_rotated_logs_keep_startup_evidence_and_cleanup_on_failure(self):
        runner = Path(__file__).resolve().parents[3] / 'tests/device/run_newton_quest_smoke.ps1'
        self.assertTrue(runner.exists(), 'bounded runner must exist')
        with tempfile.TemporaryDirectory() as temporary:
            harness = Path(temporary) / 'harness.ps1'
            harness.write_text(r'''
param($Runner, $EvidenceRoot, $Failure, $Mode = 'trace')
$ErrorActionPreference = 'Stop'
$ast = [System.Management.Automation.Language.Parser]::ParseFile($Runner, [ref]$null, [ref]$null)
$parameterProbe = [scriptblock]::Create($ast.ParamBlock.Extent.Text + "`n" + 'return $SoakSeconds')
foreach ($valid in @(0, 1, 1800, 3600)) {
    if ((& $parameterProbe -SoakSeconds $valid) -ne $valid) { throw 'Soak duration boundary did not bind' }
}
foreach ($invalid in @(-1, 3601)) {
    $rejected = $false
    try { & $parameterProbe -SoakSeconds $invalid | Out-Null } catch { $rejected = $true }
    if (-not $rejected) { throw "Soak duration $invalid must be rejected during parameter binding" }
}
foreach ($function in $ast.FindAll({param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst]}, $false)) {
    Invoke-Expression $function.Extent.Text
}
. (Join-Path (Split-Path $Runner) 'task5_checks.ps1')
$package = 'com.questnewton'
$script:poll = 0
$script:stops = 0
$script:nonce = ''
function Start-Sleep {}
function Invoke-MetaChecked([string[]] $Arguments) {
    $line = $Arguments -join ' '
    if ($line -like 'app stop *') {
        $script:stops++
        if ($Failure -eq 'cleanup' -and $script:stops -eq 2) { throw 'cleanup transport failed' }
        return ''
    }
    if ($line -eq 'shell ps -A -w -o CMD') { return "CMD`nother.application" }
    if ($line -like 'shell am start *') {
        $script:nonce = $Arguments[[array]::IndexOf($Arguments, 'quest_newton_run_id') + 1]
        $durationIndex = [array]::IndexOf($Arguments, 'quest_newton_soak_seconds')
        if ($Mode -eq 'trace' -and $durationIndex -ne -1) { throw 'Trace launch must omit the unused soak duration extra' }
        if ($Mode -eq 'soak' -and ($durationIndex -lt 1 -or $Arguments[$durationIndex - 1] -ne '--ei' -or
            $Arguments[$durationIndex + 1] -ne '1800')) { throw 'Soak launch must send --ei quest_newton_soak_seconds 1800' }
        if ($Failure -in @('launch', 'cleanup')) { throw 'transport failed after partial launch' }
        return 'Status: ok'
    }
    if ($line -eq 'shell pidof com.questnewton') { return '1234' }
    if ($line -like 'adb logcat *') {
        $script:poll++
        if ($script:poll -eq 1) { return "QUEST_NEWTON_SMOKE_OK first=0 last=.01`nQUEST_NEWTON_PASSTHROUGH_OK`nQUEST_NEWTON_OVERLAY_OK generation=1 bodies=12 visuals=11`nQUEST_NEWTON_READY mode=$Mode run_id=$script:nonce refresh_hz=90 visuals=11" }
        if ($Failure -eq 'fatal') { return 'XR_ERROR_SESSION_LOST' }
        if ($Mode -eq 'soak') { return "QUEST_NEWTON_OVERLAY_OK generation=2 bodies=12 visuals=11`nQUEST_NEWTON_SOAK_DONE run_id=$script:nonce" }
        return "QUEST_NEWTON_OVERLAY_OK generation=2 bodies=12 visuals=11`nQUEST_NEWTON_TRACE_DONE run_id=$script:nonce states=1000"
    }
    if ($line -like 'shell run-as *') { return '{"fixture":true}' }
    throw "unexpected command $line"
}
$failed = $false
try { $result = Invoke-RecordedRun -Mode $Mode -Seconds $(if ($Mode -eq 'soak') { 1800 } else { 0 }) -Name 'test' }
catch {
    $failed = $true
    if (-not $Failure) { throw }
    if ($Failure -eq 'cleanup' -and ($_.Exception.Message -notmatch 'cleanup transport failed' -or
        $_.Exception.Message -notmatch 'transport failed after partial launch')) { throw 'Both errors must be preserved' }
}
if ($script:stops -ne 2) { throw "Expected cold stop plus finally stop, got $script:stops" }
if ([bool]$Failure -ne $failed) { throw 'Failure propagation mismatch' }
if (-not $Failure) {
    if (-not (Test-Path (Join-Path $EvidenceRoot 'test-first.log'))) { throw 'first log missing' }
    if (-not (Test-Path (Join-Path $EvidenceRoot 'test-ready.log'))) { throw 'ready log missing' }
    if ((Get-Content -Raw (Join-Path $EvidenceRoot 'test-latest.log')) -match 'SMOKE_OK') { throw 'fixture did not rotate startup log' }
}
''', encoding='utf-8')
            for mode, failure in (('trace', ''), ('soak', ''), ('trace', 'fatal'), ('trace', 'launch'), ('trace', 'cleanup')):
                evidence = Path(temporary) / (mode + (failure or 'success'))
                evidence.mkdir()
                result = subprocess.run(['pwsh', '-NoProfile', '-File', str(harness), str(runner), str(evidence), failure, mode],
                                        capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
