import copy
import os
from pathlib import Path
import unittest

from tools.newton_codegen.capture_franka import build_model, load_config, validate_config, validate_state


class CaptureContractTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()

    def test_arm_order_and_fingers(self):
        self.assertEqual(self.config['joint_names'], [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7',
            'panda_finger_joint1', 'panda_finger_joint2'])
        validate_config(self.config)

    def test_duplicate_or_reordered_joint_names_rejected(self):
        for names in ([*self.config['joint_names'][:-1], 'panda_joint1'],
                      list(reversed(self.config['joint_names']))):
            candidate = copy.deepcopy(self.config)
            candidate['joint_names'] = names
            with self.assertRaises(ValueError):
                validate_config(candidate)

    def test_nonfinite_home_or_invalid_limits_rejected(self):
        for key, value in [('initial_q', float('nan')), ('initial_qd', float('inf')),
                           ('lower_limits', 100), ('upper_limits', -100)]:
            candidate = copy.deepcopy(self.config)
            candidate[key][0] = value
            with self.assertRaises(ValueError):
                validate_config(candidate)

    def test_required_high_pd_gains_cannot_be_silently_changed(self):
        for key in ('stiffness', 'damping'):
            candidate = copy.deepcopy(self.config)
            candidate[key][0] = 1
            with self.assertRaises(ValueError):
                validate_config(candidate)

    def test_required_isaac_effort_limits_cannot_be_changed(self):
        self.assertEqual(self.config['effort_limits'], [87, 87, 87, 87, 12, 12, 12, 200, 200])
        for value in (20, -1, float('nan')):
            candidate = copy.deepcopy(self.config)
            candidate['effort_limits'][8] = value
            with self.assertRaises(ValueError):
                validate_config(candidate)

    @unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'requires local Franka asset')
    def test_implicit_actuator_saturates_applied_effort_at_isaac_limits(self):
        import numpy as np
        import warp as wp
        from tools.newton_codegen.capture_franka import build_actuator, step_actuator
        wp.config.kernel_cache_dir = str(Path(__file__).resolve().parents[3] / 'out/newton-codegen-test-cache')
        model = build_model(os.environ['FRANKA_DESCRIPTION_ROOT'], self.config)
        drive = build_actuator(model, self.config)
        state, control = model.state(), model.control()
        limits = np.array([87, 87, 87, 87, 12, 12, 12, 200, 200], dtype=np.float32)
        for sign in (1., -1.):
            # Deliberately extreme target tests the drive's internal effort
            # boundary; user-facing target limits are enforced downstream.
            control.joint_target_q.assign(np.array(self.config['initial_q']) + sign * 1000)
            step_actuator(drive, state, control, .001)
            applied = control.joint_f.numpy()
            self.assertTrue(np.isfinite(applied).all())
            self.assertTrue((np.abs(applied) <= limits + 1e-4).all())
            np.testing.assert_allclose(applied, sign * limits, atol=1e-4)

    def test_arbitrary_finite_reset_state_not_clamped(self):
        validate_state([10.0] * 9, [-2.0] * 9, 9)
        for q, qd in [([0.] * 8, [0.] * 9), ([float('nan')] * 9, [0.] * 9),
                      ([0.] * 9, [float('inf')] * 9)]:
            with self.assertRaises(ValueError):
                validate_state(q, qd, 9)

    @unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'requires local Franka asset')
    def test_real_model_preserves_nine_dofs_limits_gains_and_fixed_links(self):
        import numpy as np
        import warp as wp
        wp.config.kernel_cache_dir = str(Path(__file__).resolve().parents[3] / 'out/newton-codegen-test-cache')
        model = build_model(os.environ['FRANKA_DESCRIPTION_ROOT'], self.config)
        self.assertEqual(model.joint_dof_count, 9)
        self.assertEqual(model.body_label[:2], ['panda_link0', 'panda_link1'])
        np.testing.assert_allclose(model.joint_q.numpy(), self.config['initial_q'])
        np.testing.assert_allclose(model.joint_target_ke.numpy(), [400] * 7 + [2000] * 2)
        np.testing.assert_allclose(model.joint_target_kd.numpy(), [80] * 7 + [100] * 2)
        np.testing.assert_allclose(model.gravity.numpy(), 0)
