"""Portable gripper proof must fail closed and exercise the real Session."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
# An explicit PYTHONPATH may select an integration worktree's newer runtime.
sys.path.append(str(ROOT / 'quest/app/src/main/python'))


def module():
    import quest_sim
    spec = importlib.util.spec_from_file_location('quest_sim.gripper_validation',
        ROOT / 'quest/app/src/main/python/quest_sim/gripper_validation.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class GripperValidationTests(unittest.TestCase):
    def test_unknown_diagnostic_profile_fails_closed(self):
        proof = json.loads(module().run('unused', 'unknown_model'))
        self.assertFalse(proof['passed'])
        self.assertEqual(proof['error']['type'], 'ValueError')

    def test_failed_startup_returns_strict_json_failure(self):
        validation = module()
        with mock.patch('quest_sim.runtime.Session', side_effect=RuntimeError('sentinel failure')):
            proof = json.loads(validation.run('unused'))
        self.assertFalse(proof['passed'])
        self.assertEqual(proof['error']['message'], 'sentinel failure')
        self.assertEqual(proof['checks'], [])

    @unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'real Franka assets required')
    def test_real_session_bounded_gripper_proof(self):
        proof = json.loads(module().run(os.environ['FRANKA_DESCRIPTION_ROOT']))
        self.assertTrue(proof['passed'], json.dumps(proof, indent=2))
        self.assertLessEqual(proof['step_count'], 1500)
        self.assertTrue(all(check['passed'] for check in proof['checks']))
        self.assertFalse(proof['baseline_off']['is_acceptance_gate'])
        self.assertEqual(proof['force_hold']['hold_steps'], 400)
        self.assertEqual(proof['force_hold']['desired_force_n'], 5.)
        self.assertTrue(proof['release']['opened_and_lost_finger_contact'])

    @unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'real Franka assets required')
    def test_floor_support_cannot_pass_as_table_release(self):
        validation=module(); sample=validation._sample
        def misplaced(s, initial, stage):
            row=sample(s,initial,stage)
            if stage=='release':
                row['cube_xyz']=[*row['cube_xyz'][:2],.025]
            return row
        with mock.patch.object(validation,'_sample',side_effect=misplaced):
            proof=json.loads(validation.run(os.environ['FRANKA_DESCRIPTION_ROOT']))
        self.assertFalse(proof['passed'])
        self.assertIn('table height',proof['error']['message'])

    @unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'real Franka assets required')
    def test_explicit_enhanced_profile_reports_actual_contact_dimensions_and_points(self):
        proof = json.loads(module().run(os.environ['FRANKA_DESCRIPTION_ROOT'], 'pad_manipulation_v1'))
        self.assertTrue(proof['passed'], json.dumps(proof.get('error')))
        self.assertEqual(proof['metadata']['contact_model']['profile'], 'pad_manipulation_v1')
        summary = proof['force_hold']['summary']
        self.assertEqual(summary['contact_dimensions'], [4])
        self.assertTrue(all(n > 1 for n in summary['max_finger_point_counts']))
        self.assertGreater(summary['peak_torsional_torque_abs_nm'], 0.)


if __name__ == '__main__':
    unittest.main()
