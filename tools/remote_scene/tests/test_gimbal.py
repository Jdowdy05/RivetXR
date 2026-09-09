import math
import unittest
import numpy as np
from tools.remote_scene.pose_history import PoseHistory,PanTiltHistory,PoseUnavailable
from tools.remote_scene.gimbal import RigPoseProvider,PanTiltLimits,SimulatedPanTilt


def translation(x,y,z):
    matrix=np.eye(4);matrix[:3,3]=[x,y,z];return matrix


class GimbalTests(unittest.TestCase):
    def test_measured_mount_fk_sensor_chain_and_two_exposure_mapping(self):
        robot=PoseHistory(max_gap_ns=50);angles=PanTiltHistory(max_gap_ns=50)
        robot.append(100,translation(1,0,0));robot.append(120,translation(3,0,0))
        angles.append(100,0.,0.);angles.append(120,math.pi/2,0.)
        provider=RigPoseProvider(robot,angles,robot_from_mount=translation(0,1,0),
                                 pan_from_tilt=translation(.1,0,0),gimbal_camera_id=5)
        offset=translation(.2,0,0)
        depth=provider.world_from_depth(5,120,offset)
        np.testing.assert_allclose(depth[:3,3],[3,1.3,0],atol=1e-12)
        np.testing.assert_array_equal(provider(0,120),translation(3,0,0))
        image_from_depth=translation(-.03,0,0)
        corrected=provider.image_from_depth(5,100,120,offset,image_from_depth)
        expected=image_from_depth@np.linalg.inv(provider.world_from_depth(5,120,offset))@provider.world_from_depth(5,100,offset)
        np.testing.assert_allclose(corrected,expected,atol=1e-12)
        with self.assertRaises(PoseUnavailable):provider(5,121)

    def test_motion_bound_detects_return_and_sensor_lever_arm(self):
        robot=PoseHistory(max_gap_ns=50);angles=PanTiltHistory(max_gap_ns=50)
        for stamp in (100,110,120):robot.append(stamp,np.eye(4))
        angles.append(100,0.,0.);angles.append(110,.2,0.);angles.append(120,0.,0.)
        provider=RigPoseProvider(robot,angles,robot_from_mount=np.eye(4),pan_from_tilt=np.eye(4))
        bounds=provider.motion_bounds(5,100,120,translation(.5,0,0))
        self.assertAlmostEqual(bounds.rotation_rad,.4)
        self.assertAlmostEqual(bounds.translation_m,.2)

    def test_positive_pan_looks_left_and_positive_tilt_looks_up(self):
        robot=PoseHistory(max_gap_ns=50);angles=PanTiltHistory(max_gap_ns=50)
        for stamp in (100,110,120):robot.append(stamp,np.eye(4))
        angles.append(100,0.,0.);angles.append(110,math.pi/2,0.);angles.append(120,0.,math.pi/2)
        provider=RigPoseProvider(robot,angles,robot_from_mount=np.eye(4),pan_from_tilt=np.eye(4))
        optical=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1.]])
        np.testing.assert_allclose(provider.world_from_depth(5,110,optical)[:3,2],[0,1,0],atol=1e-12)
        np.testing.assert_allclose(provider.world_from_depth(5,120,optical)[:3,2],[0,0,1],atol=1e-12)

    def test_simulation_feedback_lags_target_and_watchdog_holds(self):
        limits=PanTiltLimits(pan_min_rad=-1.,pan_max_rad=1.,tilt_min_rad=-.5,tilt_max_rad=.5,
                             max_rate_rad_s=1.,command_timeout_ns=250_000_000)
        backend=SimulatedPanTilt(limits=limits,start_ns=1_000_000_000)
        backend.set_target(.8,.4,1_000_000_000)
        sample=backend.advance(1_100_000_000)
        self.assertAlmostEqual(sample.pan_rad,.1);self.assertAlmostEqual(sample.tilt_rad,.1)
        self.assertTrue(sample.simulation_only)
        backend.advance(1_400_000_000)
        held=backend.feedback();self.assertTrue(held.command_stale)
        self.assertAlmostEqual(held.pan_rad,.25)
        backend.advance(2_000_000_000)
        self.assertEqual(backend.feedback().pan_rad,held.pan_rad)
        with self.assertRaises(ValueError):backend.set_target(2,0,2_000_000_000)
        self.assertEqual(backend.feedback().pan_rad,held.pan_rad)

    def test_tracking_loss_retires_target_and_never_fabricates_pose(self):
        backend=SimulatedPanTilt(start_ns=100)
        backend.set_target(.4,0.,100)
        backend.set_tracking(False,110)
        with self.assertRaises(PoseUnavailable):backend.history.at(110)
        with self.assertRaises(ValueError):backend.set_target(.2,0.,120)
        backend.set_tracking(True,120)
        held=backend.feedback().pan_rad
        backend.advance(130)
        self.assertEqual(backend.feedback().pan_rad,held)
        with self.assertRaises(ValueError):backend.advance(129)


if __name__=='__main__':unittest.main()
