"""Portable scene acceptance must pass real physics and fail closed on errors."""
import json
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'quest/app/src/main/python'))


class SceneValidationTests(unittest.TestCase):
    def test_failure_is_strict_json_and_never_success(self):
        from quest_sim.scene_validation import run
        with mock.patch('quest_sim.runtime.Session', side_effect=RuntimeError('sentinel failure')):
            result = json.loads(run('unused'))
        self.assertFalse(result['passed'])
        self.assertEqual(result['error']['type'], 'RuntimeError')
        self.assertEqual(result['error']['message'], 'sentinel failure')
        self.assertEqual(result['checks'], [])
        self.assertIn('validation_elapsed_seconds', result)

    @unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'real Franka assets required')
    def test_actual_scene_contract(self):
        from quest_sim.scene_validation import run
        proof = json.loads(run(os.environ['FRANKA_DESCRIPTION_ROOT']))
        self.assertTrue(proof['passed'], json.dumps(proof, indent=2))
        self.assertGreaterEqual(len(proof['checks']), 15)
        self.assertTrue(all(check['passed'] for check in proof['checks']))
        self.assertEqual(proof['physics_dt'], .005)
        self.assertEqual(proof['metadata']['backend'], 'Newton SolverMuJoCo CPU')
        self.assertGreater(proof['step_count'], 400)
        self.assertLessEqual(proof['step_count'], 1000)


if __name__ == '__main__':
    unittest.main()
