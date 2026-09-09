from types import SimpleNamespace
import unittest
import numpy as np
from tools.remote_scene.realsense import Capture,ExposureClock,extrinsic_matrix,queried_intrinsics,queried_usb_transport


class RealSenseContractTests(unittest.TestCase):
    def test_column_major_sdk_extrinsics(self):
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
        extrinsic=SimpleNamespace(rotation=rotation.ravel(order='F').tolist(),translation=[1,2,3])
        matrix=extrinsic_matrix(extrinsic)
        np.testing.assert_array_equal(matrix[:3,:3],rotation)
        np.testing.assert_array_equal(matrix[:3,3],[1,2,3])

    def test_exposure_conversion_and_wall_clock_discontinuity(self):
        clock=ExposureClock(wall_ns=100000000000,monotonic_ns=1000000000)
        # GLOBAL_TIME is the frame/readout clock; metadata adds -2 ms to the
        # middle of exposure. All cameras share this one host clock mapping.
        stamp=clock.convert(100010.,10000000,9998000,now_ns=1020000000)
        self.assertEqual(stamp,1008000000)
        with self.assertRaises(ValueError):clock.check_wall(100020000000,1000000000)
        with self.assertRaises(ValueError):clock.convert(100010.,10000000,20000000,now_ns=1020000000)
        with self.assertRaises(ValueError):clock.convert(100100.,10000000,9998000,now_ns=1020000000)

    def test_queried_distortion_is_not_silently_ignored(self):
        intr=SimpleNamespace(width=4,height=3,fx=3.,fy=3.,ppx=1.5,ppy=1.,model='none',coeffs=[0.]*5)
        self.assertEqual(queried_intrinsics(intr,'none').width,4)
        intr.model='modified_brown_conrady'
        with self.assertRaises(ValueError):queried_intrinsics(intr,'none')

    def test_wall_clock_failure_closes_direct_api_capture(self):
        class Pipeline:
            stopped=False
            def stop(self):self.stopped=True
        class Clock:
            def check_wall(self,*_):raise ValueError('wall clock jumped')
        # No SDK import or device call: exercise the public poll cleanup boundary.
        capture=Capture.__new__(Capture);pipeline=Pipeline()
        capture.rs=object();capture.pipelines=[pipeline];capture.clock=Clock()
        with self.assertRaisesRegex(RuntimeError,'capture stopped'):capture.poll()
        self.assertTrue(pipeline.stopped);self.assertEqual(capture.pipelines,[])

    def test_capture_rejects_mipi_gmsl_dds_and_unknown_transport(self):
        info=SimpleNamespace(connection_type='connection',usb_type_descriptor='usb')
        class Device:
            def __init__(self,values):self.values=values
            def supports(self,key):return key in self.values
            def get_info(self,key):return self.values[key]
        for values in ({'connection':'MIPI'},{'connection':'GMSL','usb':'3.2'},
                       {'connection':'DDS','usb':'3.2'},{'usb':'3.2'},
                       {'connection':'USB'},{'connection':'USB','usb':'Undefined'}):
            with self.subTest(values=values),self.assertRaises(ValueError):queried_usb_transport(Device(values),info)
        self.assertEqual(queried_usb_transport(Device({'connection':'USB','usb':'3.2'}),info),
                         {'connection_type':'USB','usb_type_descriptor':'3.2'})
        with self.assertRaises(ValueError):queried_usb_transport(Device({'usb':'3.2'}),SimpleNamespace(usb_type_descriptor='usb'))


if __name__=='__main__':unittest.main()
