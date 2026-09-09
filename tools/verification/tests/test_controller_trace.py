"""Portable contract checks; the real C++/Newton references run separately."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


class ControllerTraceTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('tools.verification.generate_controller_trace'),
                             'recorded trace generator must exist')
        from tools.verification import generate_controller_trace as trace
        self.trace = trace
        self.root = Path(__file__).resolve().parents[3]
        self.manifest = trace.contract_manifest()
        self.data = {'schema_version': 1, 'sample_count': 1000, 'timestep_seconds': .001,
                     'substeps_per_sample': 10, 'urdf_sha256': self.manifest['urdf_sha256'],
                     'samples': []}
        # This fixture only exercises the serialized-input parser, never physics
        # or mapper correctness. State semantics are checked by the C++ runner.
        for index in range(1000):
            phase, trigger, flags = trace.phase_input(index)
            self.data['samples'].append({'index': index, 'position': [0., 0., 0.],
                                         'rotation': [0., 0., 0., 1.], 'trigger': trigger,
                                         'flags': flags, 'phase': phase})

    def test_accepts_complete_serialized_input(self):
        self.trace.validate_trace(self.data, self.manifest)

    def test_rejects_bad_pose_trigger_index_count_flags_phase_and_hash(self):
        mutations = [lambda d: d['samples'][0]['position'].__setitem__(0, float('nan')),
                     lambda d: d['samples'][0]['position'].__setitem__(1, float('inf')),
                     lambda d: d['samples'][0]['position'].__setitem__(0, 1e100),
                     lambda d: d['samples'][0].__setitem__('rotation', [0, 0, 0, 0]),
                     lambda d: d['samples'][0].__setitem__('rotation', [0, 0, 0, 2]),
                     lambda d: d['samples'][0].__setitem__('trigger', 1.01),
                     lambda d: d['samples'][0].__setitem__('index', 1),
                     lambda d: d['samples'][0].__setitem__('flags', 1024),
                     lambda d: d['samples'][0].__setitem__('flags', True),
                     lambda d: d['samples'][0].__setitem__('phase', 'boundedinputonly'),
                     lambda d: d['samples'].pop(),
                     lambda d: d.__setitem__('sample_count', 999),
                     lambda d: d.__setitem__('urdf_sha256', '0' * 64),
                     lambda d: d['samples'][400].__setitem__('flags', 511)]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                data = copy.deepcopy(self.data)
                mutate(data)
                with self.assertRaises(ValueError):
                    self.trace.validate_trace(data, self.manifest)

    def test_rejects_changed_physics_contract(self):
        for field, value in [('joint_count', 7), ('body_count', 11), ('timestep', .002),
                             ('joint_names', list(reversed(self.manifest['joint_names']))),
                             ('warp_commit', '0' * 40)]:
            with self.subTest(field=field):
                manifest = dict(self.manifest, **{field: value})
                with self.assertRaises(ValueError):
                    self.trace.validate_trace(self.data, manifest)

    def test_generation_is_deterministic_and_copies_exact_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, manifest = root / 'input.json', root / 'artifact_manifest.json'
            source.write_text(json.dumps(self.data), encoding='utf-8')
            manifest.write_text(json.dumps(self.manifest), encoding='utf-8')
            for name in ('first', 'second'):
                self.trace.generate(source, manifest, root / name)
                self.assertEqual(source.read_bytes(), (root / name / 'controller_trace.json').read_bytes())
            first = (root / 'first/controller_trace_data.h').read_bytes()
            self.assertEqual(first, (root / 'second/controller_trace_data.h').read_bytes())
            self.assertIn(b'0.0F', first)
            self.assertNotIn(str(root).encode(), first)

    def test_tracked_trace_has_motion_and_all_required_loss_phases(self):
        source = self.root / 'tests/device/controller_trace.json'
        self.assertTrue(source.is_file(), 'canonical recorded input must be tracked')
        data = json.loads(source.read_text())
        self.trace.validate_trace(data, self.manifest)
        self.assertEqual(data['samples'][0]['position'], data['samples'][1]['position'])
        self.assertGreater(len({tuple(s['position']) for s in data['samples']}), 900)
        # Released input must still move, so a mapper that follows a released
        # grip is observable independently of the generated expected state.
        self.assertNotEqual(data['samples'][350]['position'], data['samples'][379]['position'])

    def test_real_reference_has_full_physics_state_and_rejects_wrong_rearm(self):
        self.assertIsNotNone(importlib.util.find_spec('tools.verification.generate_expected_trace'),
                             'real Newton expected generator must exist')
        from tools.verification.generate_expected_trace import validate_reference
        source = self.root / 'tests/device/expected_trace.json'
        self.assertTrue(source.is_file(), 'expected states must come from a completed real replay')
        reference = json.loads(source.read_text())
        import hashlib
        trace_hash = hashlib.sha256((self.root / 'tests/device/controller_trace.json').read_bytes()).hexdigest()
        validate_reference(reference, trace_hash, reference['physics_manifest_sha256'], self.manifest)
        self.assertEqual(reference['tolerances'], {'joint_q': 1e-5, 'joint_qd': 1e-4})
        self.assertEqual(reference['physics_substeps'], 10000)
        from tools.verification.verify_device_trace import expected_contract
        expected_contract(reference)
        self.assertNotIn('limit_violations', reference['measured'])
        self.assertGreater(reference['state_limits']['strict_excursion_samples'], 0)
        self.assertEqual(reference['state_limits']['allowance_violations'], 0)
        for field, value in (('arm_rad', 1e-4), ('finger_m', 1e-5), ('name', 'other')):
            changed = copy.deepcopy(reference)
            changed['state_limit_policy'][field] = value
            with self.subTest(policy_field=field), self.assertRaisesRegex(ValueError, 'policy'):
                validate_reference(changed, trace_hash, reference['physics_manifest_sha256'], self.manifest)
        changed = copy.deepcopy(reference)
        from tools.verification.generate_expected_trace import float32
        changed['samples'][30]['targets'][0] = float32(self.manifest['upper_limits'][0]) + 1e-8
        with self.assertRaisesRegex(ValueError, 'target outside limits'):
            validate_reference(changed, trace_hash, reference['physics_manifest_sha256'], self.manifest)
        for index in (0, 1, 350, 400, 410, 600, 610, 850, 870, 980):
            changed = copy.deepcopy(reference)
            changed['samples'][index]['engaged'] = True
            with self.subTest(index=index), self.assertRaises(ValueError):
                validate_reference(changed, trace_hash, reference['physics_manifest_sha256'], self.manifest)
        for sample in reference['samples']:
            for field in ('q', 'qd'):
                self.trace.finite_vector(sample[field], 9, field)

    def test_physics_validator_rejects_finite_but_out_of_limit_state(self):
        self.assertIsNotNone(importlib.util.find_spec('tools.verification.generate_expected_trace'),
                             'real Newton expected generator must exist')
        from tools.verification.generate_expected_trace import validate_physics_state
        q = self.manifest['initial_q'][:]
        q[0] = self.manifest['upper_limits'][0] + .000011
        with self.assertRaises(ValueError):
            validate_physics_state(q, [0.] * 9, [0.] * 84,
                                   self.manifest['lower_limits'], self.manifest['upper_limits'], 37)

    def test_state_allowances_preserve_raw_values_and_distinct_units(self):
        from tools.verification.generate_expected_trace import validate_physics_state, float32
        lower = list(map(float32, self.manifest['lower_limits']))
        upper = list(map(float32, self.manifest['upper_limits']))
        for joint, allowance in ((0, 1e-5), (7, 1e-6), (8, 1e-6)):
            for bound, sign in ((lower[joint], -1), (upper[joint], 1)):
                q = self.manifest['initial_q'][:]
                q[joint] = bound + sign * allowance * .999
                unchanged = q[:]
                with self.subTest(joint=joint, sign=sign):
                    excursions = validate_physics_state(q, [0.] * 9, [0.] * 84, lower, upper, 1)
                    self.assertAlmostEqual(excursions[joint], allowance * .999)
                    self.assertEqual(q, unchanged)
                    q[joint] = bound + sign * allowance * 1.001
                    with self.assertRaises(ValueError):
                        validate_physics_state(q, [0.] * 9, [0.] * 84, lower, upper, 2)


if __name__ == '__main__':
    unittest.main()
