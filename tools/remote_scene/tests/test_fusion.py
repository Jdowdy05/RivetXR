from dataclasses import replace
import unittest
import numpy as np
from tools.remote_scene.calibration import CameraCalibration,DepthFrame,Intrinsics,project_frame,rigid
from tools.remote_scene.fusion import fuse_batch
from tools.remote_scene.protocol import PARTIAL,TRUNCATED,pack_packet
from tools.remote_scene.demo import make_demo


def fixture(camera_id=0):
    intr=Intrinsics(3,1,1.,1.,1.,0.)
    camera=CameraCalibration(camera_id,intr,intr,.001,np.eye(4),np.eye(4))
    frame=DepthFrame(camera_id,1,1000000000,np.array([[1000,2000,0]],dtype='<u2'),np.array([[17,211,99]],dtype='u1'))
    return camera,frame


class FusionTests(unittest.TestCase):
    def test_optical_z_depth_units_and_independent_gray(self):
        camera,frame=fixture()
        points,gray=project_frame(frame,camera,'stationary')
        np.testing.assert_array_equal(points,[[-1,0,1],[0,0,2]])
        np.testing.assert_array_equal(gray,[17,211])

    def test_depth_to_intensity_extrinsic_direction(self):
        camera,frame=fixture();matrix=np.eye(4);matrix[0,3]=1
        camera=replace(camera,intensity_from_depth=matrix)
        points,gray=project_frame(frame,camera,'stationary')
        np.testing.assert_array_equal(points,[[-1,0,1],[0,0,2]])
        np.testing.assert_array_equal(gray,[211,99])

    def test_capture_time_pose_and_required_explicit_rig_mode(self):
        camera,frame=fixture();base=np.eye(4);base[2,3]=3
        camera=replace(camera,rig_from_depth=base)
        with self.assertRaises(ValueError):project_frame(frame,camera,'posed')
        motion=np.eye(4);motion[0,3]=4
        frame=replace(frame,world_from_rig=motion)
        points,_=project_frame(frame,camera,'posed')
        np.testing.assert_array_equal(points,[[3,0,4],[4,0,5]])
        with self.assertRaises(ValueError):project_frame(frame,camera,'stationary')

    def test_invalid_calibration_and_depth_rejected(self):
        camera,frame=fixture()
        for matrix in (np.zeros((4,4)),np.diag([-1,1,1,1]),np.diag([2,1,1,1])):
            with self.assertRaises(ValueError):rigid(matrix)
        with self.assertRaises(ValueError):replace(camera,depth=replace(camera.depth,distortion='brown'))
        with self.assertRaises(ValueError):project_frame(replace(frame,depth=frame.depth.astype('f4')),camera,'stationary')

    def test_skew_missing_and_invalid_depth_do_not_reuse_history(self):
        c0,f0=fixture();c1,f1=fixture(1)
        f1=replace(f1,capture_ns=f0.capture_ns+21000000)
        packet,_=fuse_batch([f0,f1],[c0,c1],produced_ns=f1.capture_ns,rig_mode='stationary')
        self.assertEqual(packet.contributing_mask,2);self.assertTrue(packet.flags&PARTIAL)
        packet,_=fuse_batch([],[c0,c1],produced_ns=f1.capture_ns+1,rig_mode='stationary')
        self.assertEqual(len(packet.points),0);self.assertEqual(packet.capture_start_ns,0)
        packet,_=fuse_batch([replace(f0,depth=np.zeros_like(f0.depth))],[c0],produced_ns=f0.capture_ns,rig_mode='stationary')
        self.assertEqual(len(packet.points),0);self.assertEqual(packet.contributing_mask,1)

    def test_support_counts_cameras_and_budget_is_deterministic(self):
        c0,f0=fixture();c1,f1=fixture(1)
        packet,_=fuse_batch([f0,f1],[c0,c1],produced_ns=f0.capture_ns,rig_mode='stationary',max_points=1)
        self.assertTrue(packet.flags&TRUNCATED);self.assertEqual(len(packet.points),1)
        self.assertEqual(packet.points['support'][0],2)
        reverse,_=fuse_batch([f1,f0],[c1,c0],produced_ns=f0.capture_ns,rig_mode='stationary',max_points=1)
        self.assertEqual(pack_packet(packet),pack_packet(reverse))

    def test_multiple_pixels_in_one_voxel_are_one_camera_support(self):
        camera,frame=fixture()
        intr=replace(camera.depth,fx=1000.)
        camera=replace(camera,depth=intr,intensity=intr)
        frame=replace(frame,depth=np.array([[1000,1000,1000]],dtype='<u2'))
        packet,_=fuse_batch([frame],[camera],produced_ns=frame.capture_ns,rig_mode='stationary')
        self.assertEqual(packet.points['support'].tolist(),[1,1])
        self.assertEqual(packet.points['gray'].tolist(),[17,155])

    def test_demo_is_five_calibrated_views_and_byte_deterministic(self):
        a,stats=make_demo(32,24);b,_=make_demo(32,24)
        self.assertEqual(a.contributing_mask,31);self.assertGreater(stats['input_points'],0)
        self.assertGreater(len(a.points),0);self.assertEqual(pack_packet(a),pack_packet(b))


if __name__=='__main__':unittest.main()
