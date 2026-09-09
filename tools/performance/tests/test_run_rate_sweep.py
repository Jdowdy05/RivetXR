"""Exercise runner races with scripted status reads; never invoke device tools."""
from collections import deque
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from tools.performance.run_rate_sweep import Sweep


class RunnerTests(unittest.TestCase):
    run_id = 'b200_0123456789ab'
    pid = '123'

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.now = 0.
        self.enterContext(patch('tools.performance.run_rate_sweep.subprocess.run',
                                side_effect=AssertionError('Device subprocess forbidden')))
        self.enterContext(patch('tools.performance.run_rate_sweep.subprocess.Popen',
                                side_effect=AssertionError('Device subprocess forbidden')))
        self.enterContext(patch('tools.performance.run_rate_sweep.uuid.uuid4',
                                return_value=SimpleNamespace(hex='0123456789abcdef')))
        self.enterContext(patch('tools.performance.run_rate_sweep.time.monotonic',
                                side_effect=lambda: self.now))
        self.enterContext(patch('tools.performance.run_rate_sweep.time.sleep',
                                side_effect=self.advance_time))
        self.enterContext(contextlib.redirect_stdout(io.StringIO()))

    def advance_time(self, seconds):
        self.now += seconds

    def status(self, phase, **changes):
        return dict(dict(run_id=self.run_id, pid=int(self.pid), phase=phase), **changes)

    def native(self, **changes):
        result = self.status('complete')
        result.update(finite=True, passed=True, requested_physics_hz=200,
                      wall_seconds=30., simulation_seconds=30., successful_steps=6000,
                      dropped_wall_seconds=0., frame_count=2700, refresh_hz=90.,
                      physics_timing=dict(count=6000, mean_us=3000, p99_us=4000))
        result.update(changes)
        return result

    def sweep(self, statuses, finals=()):
        root = Path(self.temporary.name) / str(len(list(Path(self.temporary.name).iterdir())))
        args = SimpleNamespace(output=root, metavr=Path('forbidden-device-tool'),
                               workload='motion', seconds=30, warmup=12, trace_config=None,
                               restore_live=True, leave_stopped=False, rates=[200])
        sweep = Sweep(args)
        sweep.stopped = Mock()
        sweep.call = Mock(side_effect=lambda command, **kwargs:
                          self.pid if command[:2] == ['shell', 'pidof'] else 'Status: ok')
        sweep.snapshot_health = Mock(return_value={'thermal_status': 0})
        sweep.trace = Mock(return_value=None)
        statuses, finals = deque(statuses), deque(finals)

        def read(name):
            if name == 'full-benchmark-status.json':
                if not statuses:
                    raise AssertionError('Unexpected extra status read')
                return statuses.popleft()
            self.assertEqual(name, f'full-benchmark-{self.run_id}.json')
            return finals.popleft() if finals else None

        sweep.app_json = Mock(side_effect=read)
        return sweep

    def test_first_measuring_status_rejects_before_health_collection(self):
        sweep = self.sweep([self.status('measuring')])
        with self.assertRaisesRegex(RuntimeError, 'Missed warmup'):
            sweep.trial(200, 1)
        sweep.snapshot_health.assert_not_called()
        sweep.trace.assert_not_called()
        self.assertFalse((sweep.root / 'trials.jsonl').exists())

    def test_setup_crossing_measurement_boundary_is_not_accepted(self):
        sweep = self.sweep([self.status('warmup'), self.status('measuring')])
        with self.assertRaisesRegex(RuntimeError, 'measurement boundary'):
            sweep.trial(200, 1)
        self.assertEqual(sweep.snapshot_health.call_count, 1)
        sweep.trace.assert_called_once()
        self.assertFalse((sweep.root / 'trials.jsonl').exists())

    def test_setup_requires_matching_warmup_identity(self):
        for ready in (None, self.status('warmup', run_id='stale'),
                      self.status('warmup', pid=999)):
            with self.subTest(ready=ready):
                sweep = self.sweep([self.status('warmup'), ready])
                with self.assertRaisesRegex(RuntimeError, 'measurement boundary'):
                    sweep.trial(200, 1)
                self.assertFalse((sweep.root / 'trials.jsonl').exists())

    def test_already_complete_trial_cannot_bypass_warmup_collection(self):
        sweep = self.sweep([self.status('complete')], [self.native()])
        with self.assertRaisesRegex(RuntimeError, '[Ww]armup|collection'):
            sweep.trial(200, 1)
        self.assertFalse((sweep.root / 'trials.jsonl').exists())

    def test_pre_warmup_failure_is_retained_as_failed_evidence(self):
        native = self.native(phase='failed', finite=False, passed=False,
                             failure='runtime initialization failed')
        sweep = self.sweep([self.status('failed')], [native])
        summary = sweep.trial(200, 1)
        self.assertFalse(summary['evaluation']['valid'])
        self.assertFalse(summary['evaluation']['physics_sustainable'])
        self.assertIsNone(summary['health_before'])
        retained = json.loads((sweep.root / 'trials.jsonl').read_text())
        self.assertEqual(retained['native']['failure'], 'runtime initialization failed')

    def test_terminal_status_waits_for_atomic_nonce_result(self):
        sweep = self.sweep([self.status('warmup'), self.status('warmup'),
                            self.status('complete')], [None, None, self.native()])
        summary = sweep.trial(200, 1)
        self.assertTrue(summary['evaluation']['physics_sustainable'])
        final_reads = [call for call in sweep.app_json.call_args_list
                       if call.args[0] != 'full-benchmark-status.json']
        self.assertEqual(len(final_reads), 3)
        retained = json.loads((sweep.root / 'trials.jsonl').read_text())
        self.assertEqual(retained['run_id'], self.run_id)
        self.assertIsNotNone(retained['health_before'])
        self.assertEqual(sweep.stopped.call_count, 2)

    def test_missing_final_result_times_out_without_accepting_trial(self):
        sweep = self.sweep([self.status('warmup'), self.status('warmup'),
                            self.status('complete')])
        with self.assertRaisesRegex(RuntimeError, 'Final benchmark identity missing or stale'):
            sweep.trial(200, 1)
        self.assertGreaterEqual(self.now, 10)
        self.assertLess(self.now, 12)
        self.assertFalse((sweep.root / 'trials.jsonl').exists())

    def test_wrong_final_identity_is_rejected_without_retry(self):
        for changes in ({'run_id': 'stale'}, {'pid': 999}, {'requested_physics_hz': 250}):
            with self.subTest(changes=changes):
                sweep = self.sweep([self.status('warmup'), self.status('warmup'),
                                    self.status('complete')], [self.native(**changes), self.native()])
                with self.assertRaisesRegex(RuntimeError, 'Final benchmark identity missing or stale'):
                    sweep.trial(200, 1)
                final_reads = [call for call in sweep.app_json.call_args_list
                               if call.args[0] != 'full-benchmark-status.json']
                self.assertEqual(len(final_reads), 1)
                self.assertFalse((sweep.root / 'trials.jsonl').exists())

    def test_thermal_stop_restores_proximity_without_relaunching_live(self):
        sweep = self.sweep([])
        sweep.discover = Mock()
        sweep.settings = Mock(return_value='unchanged settings')
        sweep.call = Mock(return_value='{"enabled": true}')

        def severe_trial(*args):
            sweep.thermal_stop = True
            raise RuntimeError('severe thermal throttling')

        sweep.trial = Mock(side_effect=severe_trial)
        with self.assertRaisesRegex(RuntimeError, 'severe thermal throttling'):
            sweep.run()
        commands = [call.args[0] for call in sweep.call.call_args_list]
        self.assertIn(['device', 'proximity', '--enable'], commands)
        self.assertFalse(any(command[:2] == ['app', 'launch'] for command in commands))
        sweep.stopped.assert_called_once()


if __name__ == '__main__':
    unittest.main()
