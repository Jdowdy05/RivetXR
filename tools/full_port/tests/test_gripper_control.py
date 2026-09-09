"""Pure gripper target-control contracts; no Newton or device dependency."""
from pathlib import Path
import copy
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'quest/app/src/main/python'))
from quest_sim.gripper_control import GripperController, GripperSettings


class GripperControlTests(unittest.TestCase):
    def test_off_closes_and_opens_at_physics_step_speed_bound(self):
        controller=GripperController()
        settings=GripperSettings()
        previous=(.04,.04)
        for _ in range(300):
            target=controller.step(.005,previous,None,1.,True,settings)
            self.assertTrue(all(0<=before-after<=.00025+1e-14 for before,after in zip(previous,target)))
            previous=target
        self.assertEqual(target,(0.,0.))
        self.assertEqual(controller.step(.005,(0.,0.),None,0.,True,settings),(.00025,.00025))

    def test_force_mode_acquires_contact_and_backs_off_excess_force(self):
        settings=GripperSettings(force_hold=True)
        controller=GripperController((.025,.025))
        approaching=controller.step(.005,(.025,.025),(0.,0.),1.,True,settings)
        self.assertLess(approaching[0],.025)
        backed_off=controller.step(.005,(.025,.025),(8.,6.),1.,True,settings)
        self.assertGreater(backed_off[0],approaching[0])
        self.assertAlmostEqual(backed_off[0]-approaching[0],.002*3*.005)
        self.assertEqual(controller.diagnostics()['phase'],'force_hold')
        self.assertTrue(controller.diagnostics()['contact_acquired'])
        self.assertEqual(controller.diagnostics()['desired_force_n'],5.)

    def test_one_sided_contact_uses_maximum_force_not_average(self):
        controller=GripperController((.025,.025));settings=GripperSettings(force_hold=True)
        target=controller.step(.005,(.025,.025),(8.,0.),1.,True,settings)
        self.assertGreater(target[0],.025)
        self.assertEqual(target[0],target[1])
        self.assertEqual(controller.diagnostics()['measured_max_force_n'],8.)

    def test_force_request_scales_with_analog_and_fresh_miss_remains_bounded(self):
        settings=GripperSettings(force_hold=True,max_force_n=10.)
        controller=GripperController((.025,.025))
        target=controller.step(.01,(.025,.025),(5.,4.),.5,True,settings)
        self.assertEqual(target,(.025,.025))
        target=controller.step(.01,(.025,.025),(0.,0.),.5,True,settings)
        self.assertAlmostEqual(target[0],.0249)
        self.assertTrue(controller.diagnostics()['contact_acquired'])

    def test_release_opens_even_without_contact_feedback(self):
        controller=GripperController((.02,.02));settings=GripperSettings(force_hold=True)
        controller.step(.005,(.02,.02),(5.,5.),1.,True,settings)
        target=controller.step(.005,(.02,.02),None,.02,True,settings)
        self.assertEqual(target,(.02025,.02025))
        self.assertFalse(controller.diagnostics()['contact_acquired'])
        self.assertEqual(controller.diagnostics()['phase'],'opening')

    def test_suppression_and_unavailable_feedback_freeze_without_acquisition(self):
        controller=GripperController((.022,.022));settings=GripperSettings(force_hold=True)
        held=controller.step(.005,(.022,.022),(5.,5.),1.,True,settings)
        for allowed,forces,grip in [(False,(10.,10.),1.),(False,None,0.),(True,None,1.)]:
            self.assertEqual(controller.step(.005,(.01,.01),forces,grip,allowed,settings),held)
            self.assertFalse(controller.diagnostics()['contact_acquired'])
        self.assertEqual(controller.diagnostics()['phase'],'feedback_unavailable')
        self.assertIsNone(controller.diagnostics()['measured_max_force_n'])

    def test_toggle_force_limit_and_speed_changes_do_not_jump_targets(self):
        controller=GripperController((.025,.025))
        old=controller.step(.005,(.025,.025),(5.,5.),1.,True,GripperSettings(force_hold=True))
        for settings in (GripperSettings(),GripperSettings(force_hold=True,max_force_n=.5),
                         GripperSettings(force_hold=True,max_speed_m_s=.005,max_force_n=20.)):
            target=controller.step(.005,(.025,.025),(5.,5.),1.,True,settings)
            self.assertLessEqual(abs(target[0]-old[0]),settings.max_speed_m_s*.005+1e-14)
            old=target

    def test_reset_and_model_invalidation_retire_force_feedback(self):
        controller=GripperController((.025,.025));settings=GripperSettings(force_hold=True)
        held=controller.step(.005,(.025,.025),(5.,5.),1.,True,settings)
        controller.invalidate_feedback()
        self.assertFalse(controller.diagnostics()['contact_acquired'])
        self.assertEqual(controller.step(.005,(.03,.03),None,1.,True,settings),held)
        controller.reset((.04,.04))
        self.assertEqual(controller.diagnostics()['target_q'],(.04,.04))
        controller.reset((.03,.03),target_q=(.028,.029))
        self.assertEqual(controller.step(.005,(.03,.03),None,1.,False,settings),(.028,.029))

    def test_invalid_inputs_and_settings_rejected_without_partial_state_change(self):
        controller=GripperController();settings=GripperSettings()
        before=controller.diagnostics()
        for values in ((0.,(.04,.04),None,.5,True),(.005,(float('nan'),.04),None,.5,True),
                       (.005,(.04,.04),(float('inf'),0.),.5,True),(.005,(.04,.04),(-1.,0.),.5,True),
                       (.005,(.04,.04),None,1.01,True),(.005,(.04,.04),None,.5,1)):
            with self.assertRaises(ValueError):controller.step(*values,settings)
            self.assertEqual(controller.diagnostics(),before)
        for kwargs in (dict(force_hold=1),dict(max_speed_m_s=.0049),dict(max_speed_m_s=.21),
                       dict(max_force_n=.49),dict(max_force_n=20.1),dict(max_force_n=float('nan'))):
            with self.assertRaises(ValueError):GripperSettings(**kwargs)

    def test_settings_endpoints_and_extreme_feedback_keep_target_and_rate_bounds(self):
        for speed in (.005,.2):
            for force in (.5,20.):
                controller=GripperController((0.,.04));settings=GripperSettings(True,speed,force)
                prior=(0.,.04)
                for feedback in ((10000.,0.),(0.,0.),(force,force)):
                    target=controller.step(.02,(0.,.04),feedback,1.,True,settings)
                    self.assertTrue(all(0<=q<=.04 for q in target))
                    self.assertTrue(all(abs(a-b)<=speed*.02+1e-14 for a,b in zip(target,prior)))
                    prior=target

    def test_deepcopy_retains_values_without_sharing_controller_mutation(self):
        original=GripperController((.025,.025));settings=GripperSettings(force_hold=True)
        original.step(.005,(.025,.025),(5.,5.),1.,True,settings)
        duplicate=copy.deepcopy(original)
        self.assertEqual(original.diagnostics(),duplicate.diagnostics())
        duplicate.invalidate_feedback()
        duplicate.step(.005,(.025,.025),None,0.,True,settings)
        self.assertTrue(original.diagnostics()['contact_acquired'])
        self.assertNotEqual(original.diagnostics()['target_q'],duplicate.diagnostics()['target_q'])


if __name__=='__main__':unittest.main()
