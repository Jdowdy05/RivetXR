"""Actual Newton CPU smoke. Set FRANKA_DESCRIPTION_ROOT and pinned PYTHONPATH."""
import json
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "quest/app/src/main/python"))
from quest_sim.runtime import create, HEADER, HOME


@unittest.skipUnless(os.environ.get("FRANKA_DESCRIPTION_ROOT"), "real Franka assets required")
class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.session = create(os.environ["FRANKA_DESCRIPTION_ROOT"], "{}")

    def setUp(self):
        self.session.configure('{"physics_dt":0.005,"floating_base":false}')
        self.session.command("remove_box")
        self.session.reset()

    def test_finite_steps_and_mutable_dt_reset(self):
        s = self.session
        for _ in range(40):
            snapshot = s.step(.005, HOME[:7], .3)
        self.assertEqual(HEADER.unpack_from(snapshot)[7], 40)
        self.assertAlmostEqual(s.sim_time, .2)
        s.configure('{"physics_dt":0.01}')
        s.step(.01, HOME[:7], .3)
        s.reset()
        self.assertEqual(s.sim_time, 0.)
        self.assertEqual(s.settings["physics_dt"], .01)
        s.np.testing.assert_allclose(s.state.joint_q.numpy()[s.arm_q_indices], HOME, atol=1e-6)

    def test_analog_fingers_and_invalid_configuration(self):
        s = self.session
        for closure in (0., .25, .5, .75, 1.):
            s.step(.005, HOME[:7], closure)
            s.np.testing.assert_allclose(s.control.joint_target_q.numpy()[s.arm_q_indices[-2:]], [.04 * (1 - closure)] * 2, atol=1e-7)
        before = dict(s.settings)
        with self.assertRaises(ValueError):
            s.configure('{"physics_dt":0.01,"render_interval":0}')
        self.assertEqual(s.settings, before)

    def test_box_falls_and_contacts_ground(self):
        s = self.session
        s.command("spawn_box")
        initial_z = float(s.state.body_q.numpy()[s.box_body, 2])
        contacts = 0
        for _ in range(220):
            s.step(.005, HOME[:7], 0.)
            contacts = max(contacts, int(s.solver.mj_data.ncon))
        final_z = float(s.state.body_q.numpy()[s.box_body, 2])
        self.assertLess(final_z, initial_z - .3)
        self.assertGreater(final_z, .035)
        self.assertLess(final_z, .075)
        self.assertGreater(contacts, 0)

    def test_base_motion_leaves_object_and_ground_fixed(self):
        s = self.session
        s.command("spawn_box")
        before = s.state.body_q.numpy()[s.box_body].copy()
        s.set_base_pose([0., 0., 1.2, 0., 0., 0., 1.])
        self.assertAlmostEqual(float(s.state.body_q.numpy()[0, 2]), 1.2, places=5)
        s.np.testing.assert_array_equal(s.state.body_q.numpy()[s.box_body], before)
        s.step(.005, HOME[:7], 0.)
        self.assertAlmostEqual(float(s.state.body_q.numpy()[0, 2]), 1.2, places=5)
        s.set_base_pose([0., 0., 0., 0., 0., 0., 1.])

    def test_floating_root_is_dynamic_and_cannot_be_teleported(self):
        s = self.session
        s.configure('{"floating_base":true}')
        self.assertEqual(s.model.joint_dof_count, 15)
        with self.assertRaises(ValueError):
            s.set_base_pose([0, 0, 1, 0, 0, 0, 1])
        s.step(.005, HOME[:7], 0.)


if __name__ == "__main__":
    unittest.main()
