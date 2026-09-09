"""Real Session integration of speed/force targets and transactional lifecycle."""
import json
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'quest/app/src/main/python'))
from quest_sim.runtime import Session, HOME, _settings


class GripperSettingsTests(unittest.TestCase):
    def test_legacy_defaults_and_invalid_settings(self):
        self.assertEqual(_settings('{}')['gripper_speed_mps'], 0)
        for value in [dict(gripper_force_hold=True), dict(gripper_force_hold=1),
                      dict(gripper_speed_mps=-1), dict(gripper_speed_mps=.201),
                      dict(gripper_force_n=21), dict(gripper_force_n=float('nan'))]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                _settings(json.dumps(value))


@unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'real Franka assets required')
class GripperIntegrationTests(unittest.TestCase):
    def make(self, **changes):
        options=dict(physics_dt=.005, gripper_speed_mps=.05, gripper_force_n=5,
                     gripper_force_hold=False, initial_base_pose=[0,0,1.2,0,0,0,1])
        options.update(changes)
        return Session(os.environ['FRANKA_DESCRIPTION_ROOT'], json.dumps(options))

    def targets(self, s):
        return s.control.joint_target_q.numpy()[s.arm_q_indices[-2:]].copy()

    def test_speed_is_per_physics_time_and_suppression_holds_applied_target(self):
        s=self.make(); before=self.targets(s)
        s.step(.005, HOME[:7], 1.)
        s.np.testing.assert_allclose(self.targets(s), before-.00025, atol=1e-8, rtol=0)
        held=self.targets(s)
        for _ in range(12): s.step(.005, HOME[:7], 1., False)
        s.np.testing.assert_array_equal(self.targets(s), held)
        s.configure('{"physics_dt":0.01}')
        s.step(.01, HOME[:7], 1., True)
        s.np.testing.assert_allclose(self.targets(s), held-.0005, atol=1e-8, rtol=0)

    def test_force_mode_waits_after_base_change_but_release_opens(self):
        s=self.make(gripper_force_hold=True)
        s.step(.005, HOME[:7], 1.)  # no completed solve yet: do not acquire
        s.step(.005, HOME[:7], 1.)
        held=self.targets(s)
        s.set_base_pose([.02,0,1.2,0,0,0,1])
        s.step(.005, HOME[:7], 1.)
        s.np.testing.assert_array_equal(self.targets(s), held)
        s.set_base_pose([.04,0,1.2,0,0,0,1])
        s.step(.005, HOME[:7], 0.)
        self.assertTrue(s.np.all(self.targets(s)>held))

    def test_object_room_transfer_does_not_alias_or_jump_gripper(self):
        s=self.make(gripper_force_hold=True)
        for _ in range(12): s.step(.005, HOME[:7], 1.)
        held=self.targets(s); old=s._gripper_controller
        s.command(json.dumps(dict(version=1,op='spawn',id=1,pose=[2,0,1,0,0,0,1],half_extents=[.025]*3)))
        self.assertIsNot(s._gripper_controller,old)
        s.np.testing.assert_array_equal(self.targets(s),held)
        old_target=old.diagnostics()['target_q']
        s.step(.005,HOME[:7],1.)
        s.np.testing.assert_array_equal(self.targets(s),held)
        s.step(.005,HOME[:7],1.)
        self.assertEqual(old.diagnostics()['target_q'],old_target)
        before=self.targets(s); step=s.step_index
        s.set_environment(json.dumps(dict(version=1,revision=1,enabled=True,colliders=[
            dict(kind='floor',pose=[0,0,-.025,0,0,0,1],half_extents=[3,3,.025])])) )
        self.assertEqual(s.step_index,step)
        s.np.testing.assert_array_equal(self.targets(s),before)
        s.step(.005,HOME[:7],1.)
        s.np.testing.assert_array_equal(self.targets(s),before)
        s.reset()
        s.np.testing.assert_allclose(self.targets(s),[.04,.04],atol=1e-8)

    def test_invalid_configuration_preserves_controller_and_legacy_stays_direct(self):
        s=self.make(); s.step(.005,HOME[:7],1.)
        before=self.targets(s); controller=s._gripper_controller; settings=dict(s.settings)
        with self.assertRaises(ValueError): s.configure('{"gripper_force_n":100}')
        self.assertIs(s._gripper_controller,controller); self.assertEqual(s.settings,settings)
        s.np.testing.assert_array_equal(self.targets(s),before)
        legacy=self.make(gripper_speed_mps=0)
        legacy.step(.005,HOME[:7],.75)
        legacy.np.testing.assert_allclose(self.targets(legacy),[.01,.01],atol=1e-8)
        before=self.targets(legacy)
        legacy.configure('{"gripper_speed_mps":0.05}')
        legacy.step(.005,HOME[:7],.75,False)
        legacy.np.testing.assert_array_equal(self.targets(legacy),before)

    def test_failed_reset_retires_force_feedback_before_attempt(self):
        s=self.make(gripper_force_hold=True)
        s.step(.005,HOME[:7],1.)
        self.assertTrue(s._gripper_feedback_current)
        with mock.patch.object(s.solver,'reset',side_effect=RuntimeError('reset sentinel')):
            with self.assertRaises(RuntimeError): s.reset()
        self.assertFalse(s._gripper_feedback_current)
        self.assertIn('unavailable',s.gripper_status())


if __name__=='__main__': unittest.main()
