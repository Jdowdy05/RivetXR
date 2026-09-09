import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
from tools.remote_scene.pose_history import PoseHistory,PanTiltHistory,PoseUnavailable,read_pose_trace


def pose(x=0.,angle=0.):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0,x],[s,c,0,0],[0,0,1,0],[0,0,0,1.]])


class PoseHistoryTests(unittest.TestCase):
    def test_interpolation_is_exposure_time_and_detached(self):
        history=PoseHistory(max_gap_ns=50)
        first=pose();history.append(100,first);history.append(120,pose(2,math.pi/2))
        first[0,3]=9
        np.testing.assert_allclose(history.at(110),pose(1,math.pi/4),atol=1e-12)
        np.testing.assert_array_equal(history.at(100),pose())
        result=history.at(100);result.setflags(write=True);result[0,3]=7
        np.testing.assert_array_equal(history.at(100),pose())
        for stamp in (99,121):
            with self.assertRaises(PoseUnavailable):history.at(stamp)

    def test_loss_gap_capacity_and_strict_monotonicity(self):
        history=PoseHistory(capacity=3,max_gap_ns=30)
        history.append(100,pose());history.append(110,None,tracked=False);history.append(120,pose())
        for stamp in (105,110,115):
            with self.assertRaises(PoseUnavailable):history.at(stamp)
        history.append(200,pose())
        with self.assertRaises(PoseUnavailable):history.at(100)
        with self.assertRaises(PoseUnavailable):history.at(150)
        with self.assertRaises(ValueError):history.append(200,pose())
        with self.assertRaises(ValueError):history.append(199,pose())
        np.testing.assert_array_equal(history.at(200),pose())

    def test_shortest_rotation_and_motion_bounds_include_return_motion(self):
        history=PoseHistory(max_gap_ns=50)
        history.append(100,pose(angle=math.radians(170)))
        history.append(120,pose(angle=math.radians(-170)))
        np.testing.assert_allclose(history.at(110),pose(angle=math.pi),atol=1e-12)
        joints=PanTiltHistory(max_gap_ns=50)
        joints.append(100,0.,0.);joints.append(110,.2,-.1);joints.append(120,0.,0.)
        self.assertEqual(joints.at(105),(.1,-.05))
        np.testing.assert_allclose(joints.motion_bounds(100,120),(.4,.2),atol=1e-12)
        # Measured angles must already be unwrapped; interpolation is linear.
        joints.append(140,2*math.pi,0.)
        self.assertAlmostEqual(joints.at(130)[0],math.pi)

    def test_motion_window_rejects_unsampled_gap_or_loss(self):
        history=PoseHistory(max_gap_ns=20)
        history.append(100,pose());history.append(200,pose())
        with self.assertRaises(PoseUnavailable):history.motion_bounds(100,200)
        history=PoseHistory(max_gap_ns=20)
        history.append(100,pose());history.append(110,None,tracked=False);history.append(120,pose())
        with self.assertRaises(PoseUnavailable):history.motion_bounds(100,120)

    def test_trace_is_strict_bounded_and_read_only(self):
        document=dict(version=1,clock='producer_monotonic_ns',robot=[
            dict(timestamp_ns=100,tracked=True,world_from_robot=pose().ravel().tolist()),
            dict(timestamp_ns=120,tracked=True,world_from_robot=pose(2).ravel().tolist())],
            gimbal=[dict(timestamp_ns=100,tracked=True,pan_rad=0.,tilt_rad=0.),
                    dict(timestamp_ns=120,tracked=True,pan_rad=.2,tilt_rad=0.)])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'poses.json';raw=json.dumps(document).encode();path.write_bytes(raw)
            trace=read_pose_trace(path,max_gap_ns=50)
            np.testing.assert_allclose(trace.robot.at(110),pose(1),atol=1e-12)
            self.assertEqual(trace.gimbal.at(110),(.1,0.))
            self.assertEqual(path.read_bytes(),raw)
            path.write_text(json.dumps(document).replace('"version": 1','"version": 1,"version": 2'))
            with self.assertRaises(ValueError):read_pose_trace(path)
            path.write_text(json.dumps(dict(document,clock='headset_now')))
            with self.assertRaises(ValueError):read_pose_trace(path)


if __name__=='__main__':unittest.main()
