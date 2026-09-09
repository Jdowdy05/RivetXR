"""Opt-in tests against a real host capture, using its saved CPU object modules.

Set FRANKA_ARTIFACT_DIR and pinned-source PYTHONPATH to enable these tests.
"""
import json
import os
from pathlib import Path
import unittest


@unittest.skipUnless(os.environ.get('FRANKA_ARTIFACT_DIR'), 'requires real generated graphs')
class GraphReplayTests(unittest.TestCase):
    def setUp(self):
        import warp as wp
        self.wp = wp
        self.root = Path(os.environ['FRANKA_ARTIFACT_DIR'])
        self.metadata = json.loads((self.root / 'capture_metadata.json').read_text())
        wp.config.kernel_cache_dir = str(self.root / '_build/test-cache')
        wp.init()

    def array(self, data):
        return self.wp.array(data, dtype=self.wp.float32, device='cpu')

    def test_exact_reset_and_step_buffer_names_and_sizes(self):
        reset = {'joint_q_in': 36, 'joint_qd_in': 36, 'joint_q_out': 36,
                 'joint_qd_out': 36, 'body_q_out': 336}
        step = dict(reset, joint_force=36, joint_target_q=36, joint_target_qd=36)
        for kind, expected in [('reset', reset), ('step', step)]:
            graph = self.wp.capture_load(str(self.root / f'franka_{kind}.wrp'), device='cpu')
            self.assertEqual({name: value['size'] for name, value in graph._params.items()}, expected)

    def test_nonzero_force_velocity_target_and_step_body_outputs(self):
        import numpy as np
        import newton
        from tools.newton_codegen.capture_franka import build_model, load_config
        graph = self.wp.capture_load(str(self.root / 'franka_step.wrp'), device='cpu')
        zero = [0.] * 9
        home = self.metadata['initial_q']
        model = build_model(os.environ['FRANKA_DESCRIPTION_ROOT'], load_config())

        def one_step(force, target_qd=zero, qd_in=zero):
            # Restore the same input state each time. This detects accumulating
            # actuator effort and mutation/aliasing of the external force input.
            for name, values in [('joint_q_in', home), ('joint_qd_in', qd_in),
                                 ('joint_target_q', home), ('joint_target_qd', target_qd),
                                 ('joint_force', force)]:
                graph.set_param(name, self.array(values))
            self.wp.capture_launch(graph)
            q, qd, preserved_force = self.array(zero), self.array(zero), self.array(zero)
            bodies = self.wp.zeros(12, dtype=self.wp.transform, device='cpu')
            graph.get_param('joint_q_out', q)
            graph.get_param('joint_qd_out', qd)
            graph.get_param('joint_force', preserved_force)
            graph.get_param('body_q_out', bodies)
            np.testing.assert_array_equal(preserved_force.numpy(), np.array(force, dtype=np.float32))
            for values in (q.numpy(), qd.numpy(), bodies.numpy()):
                self.assertTrue(np.isfinite(values).all())
            state = model.state()
            newton.eval_fk(model, q, qd, state)
            np.testing.assert_allclose(bodies.numpy(), state.body_q.numpy(), atol=1e-6)
            return q.numpy(), qd.numpy()

        q0, v0 = one_step(zero)
        np.testing.assert_array_equal(v0, np.zeros(9, dtype=np.float32))
        np.testing.assert_allclose(q0, home, atol=1e-7)
        force = [.01] + [0.] * 8
        qp, vp = one_step(force)
        _, vn = one_step([-.01] + [0.] * 8)
        self.assertGreater(vp[0], 0.)
        np.testing.assert_allclose(vn, -vp, rtol=1e-4, atol=1e-8)
        _, repeated = one_step(force)
        np.testing.assert_array_equal(repeated, vp)
        _, cleared = one_step(zero)
        np.testing.assert_array_equal(cleared, v0)
        _, target_response = one_step(zero, target_qd=[.1] + [0.] * 8)
        self.assertGreater(target_response[0], 0.)
        self.assertGreater(np.linalg.norm(target_response - v0), 1e-5)
        _, velocity_response = one_step(zero, qd_in=[.01] + [0.] * 8)
        self.assertGreater(np.linalg.norm(velocity_response - v0), 1e-5)

    def test_reset_preserves_arbitrary_finite_state_and_evaluates_fk(self):
        import newton
        import numpy as np
        from tools.newton_codegen.capture_franka import build_model, load_config
        graph = self.wp.capture_load(str(self.root / 'franka_reset.wrp'), device='cpu')
        q = self.array([0.1, -0.5, 0.2, -2.7, 0.3, 3.0, 0.8, 0.03, 0.02])
        qd = self.array([0.1] * 9)
        graph.set_param('joint_q_in', q)
        graph.set_param('joint_qd_in', qd)
        self.wp.capture_launch(graph)
        actual_q, actual_qd = self.array([0] * 9), self.array([0] * 9)
        graph.get_param('joint_q_out', actual_q)
        graph.get_param('joint_qd_out', actual_qd)
        np.testing.assert_array_equal(actual_q.numpy(), q.numpy())
        np.testing.assert_array_equal(actual_qd.numpy(), qd.numpy())
        bodies = self.wp.zeros(12, dtype=self.wp.transform, device='cpu')
        graph.get_param('body_q_out', bodies)
        model = build_model(os.environ['FRANKA_DESCRIPTION_ROOT'], load_config())
        state = model.state()
        newton.eval_fk(model, q, qd, state)
        np.testing.assert_allclose(bodies.numpy(), state.body_q.numpy(), atol=1e-6)

    def test_home_pose_remains_finite_for_ten_thousand_steps(self):
        import numpy as np
        graph = self.wp.capture_load(str(self.root / 'franka_step.wrp'), device='cpu')
        q, qd = self.array(self.metadata['initial_q']), self.array([0] * 9)
        graph.set_param('joint_q_in', q)
        graph.set_param('joint_qd_in', qd)
        graph.set_param('joint_target_q', q)
        graph.set_param('joint_target_qd', qd)
        graph.set_param('joint_force', qd)
        for index in range(10000):
            self.wp.capture_launch(graph)
            graph.get_param('joint_q_out', q)
            graph.get_param('joint_qd_out', qd)
            self.assertTrue(np.isfinite(q.numpy()).all(), f'q diverged at {index}')
            self.assertTrue(np.isfinite(qd.numpy()).all(), f'qd diverged at {index}')
            graph.set_param('joint_q_in', q)
            graph.set_param('joint_qd_in', qd)
        np.testing.assert_allclose(q.numpy(), self.metadata['initial_q'], atol=1e-6)

    def test_valid_arm_and_finger_motion_remains_bounded_and_tracks_target(self):
        import numpy as np
        graph = self.wp.capture_load(str(self.root / 'franka_step.wrp'), device='cpu')
        target = np.array(self.metadata['initial_q'], dtype=np.float32)
        target += np.array([.15, .05, -.1, .1, .05, -.1, -.1, -.015, -.015], dtype=np.float32)
        graph.set_param('joint_target_q', self.array(target))
        q, qd = self.array(self.metadata['initial_q']), self.array([0] * 9)
        graph.set_param('joint_q_in', q)
        graph.set_param('joint_qd_in', qd)
        for index in range(1500):
            self.wp.capture_launch(graph)
            graph.get_param('joint_q_out', q)
            graph.get_param('joint_qd_out', qd)
            self.assertTrue(np.isfinite(q.numpy()).all(), f'valid motion q diverged at {index}')
            self.assertTrue(np.isfinite(qd.numpy()).all(), f'valid motion qd diverged at {index}')
            self.assertTrue((q.numpy() >= np.array(self.metadata['lower_limits']) - .001).all())
            self.assertTrue((q.numpy() <= np.array(self.metadata['upper_limits']) + .001).all())
            graph.set_param('joint_q_in', q)
            graph.set_param('joint_qd_in', qd)
        np.testing.assert_allclose(q.numpy(), target, atol=.001)
