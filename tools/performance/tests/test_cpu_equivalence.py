"""Host-only contracts for the exact CPU reference/candidate replay checker."""
import struct
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tools.performance import verify_cpu_equivalence as checker
from tools.performance.verify_cpu_equivalence import (
    first_difference, operation_schedule, snapshot_physics_bytes,
)


class CpuEquivalenceTests(unittest.TestCase):
    def test_host_and_android_use_the_same_comparison_functions(self):
        self.assertIs(checker.first_difference, checker.cpu_validation.first_difference)
        self.assertIs(checker.compare_replay, checker.cpu_validation.compare_replay)

    def test_portable_reference_control_failure_blocks_candidate(self):
        portable = checker.cpu_validation
        with patch.object(portable, 'compare_replay', return_value={'passed': False}) as compare:
            proof = portable.validate(object(), object(), 'assets', steps=8)
        self.assertFalse(proof['passed'])
        self.assertNotIn('candidate_comparison', proof)
        compare.assert_called_once()

    def test_portable_comparison_exception_is_returned_as_failed_proof(self):
        portable = checker.cpu_validation
        with patch.object(portable, 'compare_replay', side_effect=RuntimeError('solver failed')):
            proof = portable.validate(object(), object(), 'assets', steps=8)
        self.assertFalse(proof['passed'])
        self.assertEqual(proof['error']['message'], 'solver failed')

    def test_portable_candidate_mismatch_remains_failed_proof(self):
        portable = checker.cpu_validation
        mismatch = {'passed': False, 'difference': {'field': 'state.joint_q'}}
        with patch.object(portable, 'compare_replay', side_effect=[{'passed': True}, mismatch]):
            proof = portable.validate(object(), object(), 'assets', steps=8)
        self.assertFalse(proof['passed'])
        self.assertEqual(proof['candidate_comparison']['difference']['field'], 'state.joint_q')

    def test_android_entry_uses_explicit_upstream_and_cached_factories(self):
        portable = checker.cpu_validation
        modes = []
        runtime = SimpleNamespace(Session=lambda assets, settings, *, cpu_cache:
                                  modes.append(cpu_cache))
        def compare(reference, candidate, assets, operations):
            reference(assets, '{}'); candidate(assets, '{}')
            return {'passed': True}
        with patch.dict('sys.modules', {'quest_sim.runtime': runtime}), \
                patch.object(portable, 'compare_replay', side_effect=compare):
            proof = json.loads(portable.run('assets', 8))
        self.assertTrue(proof['passed'])
        self.assertEqual(modes, [False, False, False, True])

    def test_java_validation_is_opt_in_and_written_before_rejection(self):
        source = (checker.REPO / 'quest/app/src/full/java/com/questnewton/SimulationBridge.java').read_text()
        self.assertIn('getBooleanExtra("quest_newton_cpu_verify", false)', source)
        proof = source.index('String proof =')
        write = source.index('writeRequiredEvidence(application, "full-cpu-equivalence.json", proof)', proof)
        reject = source.index('getBoolean("passed")', write)
        create = source.index('session = runtime.callAttr("create"', reject)
        self.assertLess(proof, write)
        self.assertLess(write, reject)
        self.assertLess(reject, create)
        self.assertIn('throw new IOException', source[reject:create])

    def snapshot(self, step=7, elapsed=2.5):
        return struct.pack('<4sHHIIIIQdd', b'QSIM', 1, 48, 12, 0, 1, 3,
                           step, step * .005, elapsed) + bytes(12 * 28)

    def test_only_last_step_wall_timing_is_excluded(self):
        self.assertEqual(snapshot_physics_bytes(self.snapshot(elapsed=1)),
                         snapshot_physics_bytes(self.snapshot(elapsed=99)))
        self.assertNotEqual(snapshot_physics_bytes(self.snapshot(step=7)),
                            snapshot_physics_bytes(self.snapshot(step=8)))
        changed = bytearray(self.snapshot()); changed[-1] = 1
        self.assertNotEqual(snapshot_physics_bytes(self.snapshot()),
                            snapshot_physics_bytes(bytes(changed)))

    def test_malformed_snapshot_does_not_become_equivalent(self):
        for data in (b'', b'QSIM', bytes(48), self.snapshot()[:-1]):
            with self.subTest(length=len(data)), self.assertRaises(ValueError):
                snapshot_physics_bytes(data)

    def test_signed_zero_and_one_bit_changes_are_not_tolerated(self):
        reference = {'q': ('<f8', (1,), struct.pack('<d', 0.))}
        candidate = {'q': ('<f8', (1,), struct.pack('<d', -0.))}
        self.assertIsNotNone(first_difference(reference, candidate))
        candidate['q'] = ('<f8', (1,), b'\x01' + bytes(7))
        self.assertIsNotNone(first_difference(reference, candidate))
        self.assertIsNone(first_difference(reference, dict(reference)))

    def test_missing_field_shape_and_exception_changes_are_detected(self):
        reference = {'raw': ('<f4', (2,), bytes(8)), 'error': ('ValueError', 'bad target')}
        for candidate in ({'raw': reference['raw']},
                          dict(reference, raw=('<f4', (1, 2), bytes(8))),
                          dict(reference, error=('ValueError', 'different error'))):
            self.assertIsNotNone(first_difference(reference, candidate))

    def test_schedule_exercises_required_physics_and_failure_paths(self):
        ops = operation_schedule(8)
        kinds = {op['kind'] for op in ops}
        self.assertTrue({'step', 'solver_step', 'base', 'configure', 'command',
                         'forces', 'inject', 'snapshot'} <= kinds)
        configurations = [op['settings'] for op in ops if op['kind'] == 'configure']
        self.assertTrue(any(s.get('floating_base') is True for s in configurations))
        self.assertTrue(any(s.get('gravity_scale') == .5 for s in configurations))
        self.assertTrue(any(s.get('physics_dt') == .0025 for s in configurations))
        self.assertEqual({op['name'] for op in ops if op['kind'] == 'command'},
                         {'spawn_box', 'remove_box', 'reset'})
        injections = [op for op in ops if op['kind'] == 'inject']
        self.assertTrue(any(op['array'] == 'joint_qd' for op in injections))
        self.assertTrue(any(op['array'] == 'body_q' for op in injections))
        self.assertTrue(any(op.get('expect_error') for op in ops))

    def test_timing_failure_never_produces_successful_cli_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'evidence.json'
            asset = Path(directory) / 'robots' / 'panda_arm_hand.urdf'
            asset.parent.mkdir(); asset.write_text('<robot/>')
            argv = ['verify_cpu_equivalence', '--franka-description-root', directory,
                    '--output', str(output), '--steps', '8', '--timing-steps', '8']
            with patch('sys.argv', argv), \
                    patch.object(checker.subprocess, 'check_output', side_effect=[b'pass', 'commit']), \
                    patch.object(checker, '_load_module', return_value=SimpleNamespace(create=object())), \
                    patch.object(checker, 'compare_replay', return_value={'passed': True}), \
                    patch.object(checker, 'timing_pass', side_effect=RuntimeError('timing failed')), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(checker.main(), 1)
            saved = json.loads(output.read_text())
            self.assertFalse(saved['passed'])
            self.assertTrue(saved['candidate_comparison']['passed'])
            self.assertEqual(saved['error']['message'], 'timing failed')


if __name__ == '__main__':
    unittest.main()
