from copy import deepcopy
import sys
from types import SimpleNamespace
import unittest
from unittest import mock
import numpy as np
from tools.remote_scene.realsense import Capture,RgbRectifier,rgb_motion_gate,_validate_config
from tools.remote_scene.calibration import CameraCalibration,Intrinsics
from tools.remote_scene.gimbal import MotionBounds
from tools.remote_scene.pose_history import PoseUnavailable


def sdk():
    calls=[]
    def project(intr,point):
        calls.append(tuple(point))
        return [point[0]/point[2]*intr.fx+intr.ppx+intr.coeffs[0],point[1]/point[2]*intr.fy+intr.ppy]
    value=SimpleNamespace(distortion=SimpleNamespace(none='none',brown_conrady='brown',modified_brown_conrady='modified'),
                          intrinsics=SimpleNamespace,rs2_project_point_to_pixel=project)
    return value,calls


def intrinsics(model='none',shift=0.):
    return SimpleNamespace(width=4,height=3,fx=3.,fy=3.,ppx=1.5,ppy=1.,model=model,coeffs=[shift,0,0,0,0])


def config():
    return dict(version=2,sdk_version='test',source_id=1,world_epoch=1,calibration_id=1,rig_mode='posed',
                observer_pose=[0,0,0,0,0,0,1],max_pair_skew_ms=5,pose=None,cameras=[
        dict(camera_id=5,serial='private-test-camera',depth=dict(width=4,height=3,fps=30,index=0),
             intensity=dict(width=4,height=3,fps=30,index=0),rig_from_depth=np.eye(4).ravel().tolist(),
             image_format='RGB8',rgb_policy=dict(support_window_ms=None,max_translation_m=.001,max_rotation_rad=.005))])


class RgbCaptureContractTests(unittest.TestCase):
    def test_cached_sdk_lookup_rectifies_and_crops_invalid_border(self):
        rs,calls=sdk();native=intrinsics('brown',1.)
        rectifier=RgbRectifier(rs,native)
        image=np.arange(36,dtype=np.uint8).reshape(3,4,3)
        before=len(calls);output=rectifier.apply(image,native)
        np.testing.assert_array_equal(output,image[:,1:,:])
        self.assertEqual((rectifier.intrinsics.width,rectifier.intrinsics.height),(3,3))
        self.assertEqual(rectifier.intrinsics.distortion,'none')
        self.assertEqual(rectifier.intrinsics.ppx,1.5)
        self.assertLessEqual(len(calls)-before,9)
        image[:]=0
        self.assertTrue(np.any(output))

    def test_native_profile_change_or_sdk_lookup_change_rejects(self):
        rs,_=sdk();native=intrinsics();rectifier=RgbRectifier(rs,native)
        changed=intrinsics();changed.fx=4.
        image=np.zeros((3,4,3),dtype=np.uint8)
        with self.assertRaises(ValueError):rectifier.apply(image,changed)
        rs.rs2_project_point_to_pixel=lambda *_:[999.,999.]
        with self.assertRaises(ValueError):rectifier.apply(image,native)
        with self.assertRaises(ValueError):RgbRectifier(rs,intrinsics('inverse'))

    def test_unknown_readout_and_excess_motion_mark_image_unusable(self):
        i=Intrinsics(4,3,3.,3.,1.5,1.)
        camera=CameraCalibration(5,i,i,.001,np.eye(4),np.eye(4),'RGB8')
        policy=config()['cameras'][0]['rgb_policy']
        class Provider:
            def __init__(self,motion):self.motion=motion;self.window=None
            def motion_bounds(self,camera_id,start,end,sensor):self.window=(camera_id,start,end);return self.motion
        provider=Provider(MotionBounds(0,0))
        self.assertFalse(rgb_motion_gate(camera,100_000_000,102_000_000,provider,policy)[0])
        policy=dict(policy,support_window_ms=5.)
        self.assertTrue(rgb_motion_gate(camera,100_000_000,102_000_000,provider,policy)[0])
        self.assertEqual(provider.window,(5,95_000_000,107_000_000))
        self.assertFalse(rgb_motion_gate(camera,100_000_000,102_000_000,Provider(MotionBounds(.1,0)),policy)[0])
        self.assertFalse(rgb_motion_gate(camera,100_000_000,102_000_000,None,policy)[0])
        self.assertFalse(rgb_motion_gate(camera,100_000_000,102_000_000,Provider(None),policy)[0])
        self.assertFalse(rgb_motion_gate(camera,100_000_000,102_000_000,Provider(MotionBounds(float('nan'),0)),policy)[0])
        class Unsupported:
            def motion_bounds(self,*_):raise NotImplementedError
        self.assertFalse(rgb_motion_gate(camera,100_000_000,102_000_000,Unsupported(),policy)[0])

    def test_rgb_motion_gate_includes_color_sensor_lever_arm(self):
        i=Intrinsics(4,3,3.,3.,1.5,1.);offset=np.eye(4);offset[0,3]=-1.
        camera=CameraCalibration(5,i,i,.001,np.eye(4),offset,'RGB8')
        class Provider:
            def motion_bounds(self,_,start,end,sensor):return MotionBounds(float(np.linalg.norm(sensor[:3,3]))*.1,.1)
        policy=dict(support_window_ms=5.,max_translation_m=.05,max_rotation_rad=.2)
        self.assertFalse(rgb_motion_gate(camera,100_000_000,102_000_000,Provider(),policy)[0])

    def test_v2_supports_six_and_rgb_but_v1_contract_stays_five_y8(self):
        value=config();_validate_config(value)
        for camera_id in range(5):
            camera=deepcopy(value['cameras'][0]);camera.update(camera_id=camera_id,serial=f'private-{camera_id}',image_format='Y8',rgb_policy=None)
            value['cameras'].append(camera)
        _validate_config(value)
        value['cameras'][0]['camera_id']=6
        with self.assertRaises(ValueError):_validate_config(value)
        with self.assertRaises(ValueError):_validate_config(dict(config(),version=1))

    def test_mocked_rgb_capture_has_separate_clock_pose_and_depth_only_fallback(self):
        rs,_=sdk();rs.stream=SimpleNamespace(depth='depth',color='color',infrared='ir')
        rs.format=SimpleNamespace(z16='z16',rgb8='rgb8',y8='y8')
        rs.option=SimpleNamespace(global_time_enabled='global_time')
        rs.camera_info=SimpleNamespace(product_line='line',product_id='pid',connection_type='connection',
            usb_type_descriptor='usb',firmware_version='firmware')
        rs.frame_metadata_value=SimpleNamespace(frame_timestamp='frame',sensor_timestamp='sensor')
        rs.timestamp_domain=SimpleNamespace(global_time='global')
        class Sensor:
            def __init__(self):self.options={}
            def supports(self,key):return key=='global_time'
            def set_option(self,key,value):self.options[key]=value
            def get_option(self,key):return self.options[key]
            def get_depth_scale(self):return .001
        depth_sensor=Sensor();rgb_sensor=Sensor()
        class Device:
            def supports(self,_):return True
            def get_info(self,key):return {'line':'D400','pid':'0B3A','connection':'USB','usb':'3.2','firmware':'test'}[key]
            def first_depth_sensor(self):return depth_sensor
            def first_color_sensor(self):return rgb_sensor
        device=Device()
        class Profile:
            def __init__(self,stream,args):self.stream=stream;self.args=args
            def as_video_stream_profile(self):return self
            def width(self):return self.args[1]
            def height(self):return self.args[2]
            def fps(self):return self.args[4]
            def stream_index(self):return self.args[0]
            def format(self):return self.args[3]
            def unique_id(self):return 1 if self.stream=='depth' else 2
            def get_intrinsics(self):return intrinsics()
            def get_extrinsics_to(self,_):return SimpleNamespace(rotation=np.eye(3).ravel().tolist(),translation=[.03,0,0])
        class Active:
            def __init__(self,configuration):self.configuration=configuration
            def get_device(self):return device
            def get_stream(self,stream,_):return Profile(stream,self.configuration.requests[stream])
        class Configuration:
            def __init__(self):self.requests={}
            def enable_device(self,_):pass
            def enable_stream(self,stream,*args):self.requests[stream]=args
            def resolve(self,_):return Active(self)
        class Pipeline:
            def __init__(self):self.pending=[];self.stopped=False
            def start(self,cfg):return Active(cfg)
            def poll_for_frames(self):return self.pending.pop(0) if self.pending else None
            def stop(self):self.stopped=True
        pipeline=Pipeline();rs.pipeline=lambda:pipeline;rs.pipeline_wrapper=lambda p:p;rs.config=Configuration
        class Frame:
            def __init__(self,profile,stamp,number):self.profile=profile;self.stamp=stamp;self.number=number
            def get_frame_number(self):return self.number
            def get_frame_timestamp_domain(self):return 'global'
            def supports_frame_metadata(self,_):return True
            def get_frame_metadata(self,key):return (self.stamp+(1_000_000 if key=='frame' else 0))//1000
            def get_timestamp(self):return (100_000_000_000+self.stamp+1_000_000)/1_000_000
            def get_data(self):return np.full((3,4),1000,dtype='<u2') if self.profile.stream=='depth' else np.arange(36,dtype=np.uint8).reshape(3,4,3)
        class FrameSet:
            def __init__(self,depth,image):self.depth=depth;self.image=image
            def get_depth_frame(self):return self.depth
            def get_color_frame(self):return self.image
        class Provider:
            reject=False
            def __call__(self,_,stamp):
                if self.reject:raise PoseUnavailable('fixture tracking loss')
                value=np.eye(4);value[0,3]=(stamp-1_000_000_000)/1e9;return value
            def motion_bounds(self,_,start,end,sensor):return MotionBounds((end-start)/1e9,0.)
        provider=Provider();now=[1_500_000_000]
        value=config();value['cameras'][0]['rgb_policy']=dict(support_window_ms=5.,max_translation_m=.1,max_rotation_rad=.005)
        with mock.patch.dict(sys.modules,{'pyrealsense2':rs}),mock.patch('tools.remote_scene.realsense.importlib.metadata.version',return_value='test'),\
             mock.patch('tools.remote_scene.realsense.time.monotonic_ns',side_effect=lambda:now[0]),\
             mock.patch('tools.remote_scene.realsense.time.time_ns',side_effect=lambda:100_000_000_000+now[0]):
            capture=Capture(value,provider).open()
            self.assertEqual(depth_sensor.options,{'global_time':1.})
            self.assertEqual(rgb_sensor.options,{'global_time':1.})
            dp,ip=capture.profiles[0]
            pipeline.pending=[FrameSet(Frame(dp,1_400_000_000,1),Frame(ip,1_402_000_000,1))]
            first=capture.poll()[0]
            self.assertEqual((first.capture_ns,first.image_timestamp_ns),(1_400_000_000,1_402_000_000))
            self.assertTrue(first.image_usable)
            self.assertAlmostEqual(first.image_from_depth[0,3],.028)
            first.intensity[:]=0
            now[0]=5_000_000_000
            pipeline.pending=[FrameSet(Frame(dp,4_950_000_000,2),None)]
            stale=capture.poll()[0]
            self.assertFalse(stale.image_usable)
            self.assertEqual(stale.image_timestamp_ns,1_402_000_000)
            self.assertTrue(np.any(stale.intensity))
            self.assertEqual(stale.capture_ns,4_950_000_000)
            provider.reject=True
            pipeline.pending=[FrameSet(Frame(dp,4_970_000_000,3),None)]
            self.assertEqual(capture.poll(),[]);self.assertFalse(capture.tracking_valid)
            capture.close();self.assertTrue(pipeline.stopped)
            self.assertFalse(capture.last_images);self.assertFalse(capture.rectifiers)


if __name__=='__main__':unittest.main()
