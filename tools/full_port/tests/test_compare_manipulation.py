"""Metric semantics for deterministic manipulation profile comparisons."""
import unittest
import json
from pathlib import Path
import tempfile
from tools.full_port.compare_manipulation import stage_summary, quality_checks, settings_for_mode, compare_reports


def contact(finger=None, force=3., distance=-.002, role='cube_finger', condim=3, torque=0.):
    return dict(finger=finger, force6=[force,0.,0.,torque,0.,0.], distance_m=distance,
                role=role, condim=condim, geom_a=1, geom_b=2, position=[0.,0.,1.])


def row(z=1.025, contacts=None, hand_z=1., relative=(0.,0.,.025)):
    return dict(cube_pose=[0.,0.,z,0.,0.,0.,1.],hand_pose=[0.,0.,hand_z,0.,0.,0.,1.],
                cube_in_hand=list(relative),finger_q=[.04,.04],finger_target_q=[.04,.04],
                contacts=contacts or [],ncon=4,step_wall_us=200.,sample_wall_us=100.)


class ManipulationMetricsTests(unittest.TestCase):
    def test_explicit_mode_changes_only_force_enable_and_rejects_unknown(self):
        force=settings_for_mode('force');position=settings_for_mode('position')
        self.assertTrue(force['gripper_force_hold']);self.assertFalse(position['gripper_force_hold'])
        self.assertEqual({k:v for k,v in force.items() if k!='gripper_force_hold'},
                         {k:v for k,v in position.items() if k!='gripper_force_hold'})
        self.assertEqual(position['gripper_speed_mps'],.05)
        self.assertEqual(position['physics_dt'],.005)
        with self.assertRaises(ValueError):settings_for_mode('unknown')

    def test_combiner_rejects_pooling_different_gripper_modes(self):
        with tempfile.TemporaryDirectory() as directory:
            paths=[]
            for mode in ('force','position'):
                path=Path(directory)/(mode+'.json');paths.append(path)
                path.write_text(json.dumps(dict(schema_version=1,fixtures=[],settings=settings_for_mode(mode))))
            with self.assertRaisesRegex(ValueError,'different controller/step settings'):compare_reports(paths)

    def test_zero_force_distance_does_not_claim_force_bearing_penetration(self):
        r=row(contacts=[contact(0,0.,-.03),contact(1,4.,-.002)])
        result=stage_summary([r],[0.,0.,1.025],[0.,0.,.025])
        self.assertAlmostEqual(result['minimum_active_distance_m'],-.03)
        self.assertAlmostEqual(result['minimum_force_bearing_distance_m'],-.002)
        self.assertEqual(result['bilateral_samples'],0)

    def test_torque_and_multiple_contact_points_are_explicit(self):
        r=row(contacts=[contact(0,condim=4,torque=.01),contact(0,force=2.),contact(1)])
        result=stage_summary([r],[0.,0.,1.025],[0.,0.,.025])
        self.assertEqual(result['peak_finger_point_counts'],[2,1])
        self.assertEqual(result['peak_finger_normal_sum_n'],[5.,3.])
        self.assertEqual(result['contact_dimensions'],[3,4])
        self.assertAlmostEqual(result['peak_torsional_torque_abs_nm'],.01)

    def test_lift_requires_bilateral_unsupported_persistence_not_only_peak(self):
        supported=[row(contacts=[contact(0),contact(1),contact(role='table')]) for _ in range(100)]
        lifted=[row(z=1.08,hand_z=1.06,contacts=[contact(0),contact(1)],relative=(0,0,.02)) for _ in range(100)]
        release=[row(contacts=[contact(role='table')]) for _ in range(20)]
        result=quality_checks({'supported_hold':supported,'lift_hold':lifted,'release':release},1.,[0.,0.,1.025])
        self.assertTrue(result['sustained_unsupported_lift'])
        lifted[-1]=row(z=1.025,contacts=[contact(role='table')])
        result=quality_checks({'supported_hold':supported,'lift_hold':lifted,'release':release},1.,[0.,0.,1.025])
        self.assertFalse(result['sustained_unsupported_lift'])

    def test_floor_support_cannot_pass_release_on_table(self):
        release=[row(z=.025,contacts=[contact(role='floor')]) for _ in range(20)]
        result=quality_checks({'release':release},1.,[0.,0.,1.025])
        self.assertFalse(result['released_on_table'])
        release=[row(contacts=[contact(role='floor')]) for _ in range(20)]
        self.assertFalse(quality_checks({'release':release},1.,[0.,0.,1.025])['released_on_table'])

    def test_a_baseline_without_success_is_still_a_valid_diagnostic(self):
        result=quality_checks({'supported_hold':[row()],'release':[row()]},1.,[0.,0.,1.025])
        self.assertFalse(result['supported_bilateral_hold'])
        self.assertFalse(result['released_on_table'])
        self.assertIsNone(result['sustained_unsupported_lift'])


if __name__=='__main__':unittest.main()
