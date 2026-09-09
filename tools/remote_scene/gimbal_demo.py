"""Six-camera synthetic source driven by measured simulation feedback only."""
import math
import secrets
from threading import Event
import time
import numpy as np
from .demo import camera_pose,OBSERVER
from .gimbal import SimulatedPanTilt
from .rgb_demo import six_camera_batch
from .rgb_surface import build_rgb_batch
from .observation_map import RetainedObservationMap

SOURCE_ID=0x47524c44454d4f01
STATIC_SOURCE_ID=0x47524c44454d4f02


def new_live_source_id():
    # Pair scene and control from this process only; reserved static/demo IDs
    # must never authorize an unrelated interactive simulation instance.
    for _ in range(16):
        value=secrets.randbits(64)
        if value not in (0,SOURCE_ID,STATIC_SOURCE_ID):return value
    raise RuntimeError('could not allocate a live simulation source identity')


def optical_pose(pan_rad,tilt_rad):
    if not math.isfinite(pan_rad) or not math.isfinite(tilt_rad):raise ValueError('finite simulated angles required')
    base=camera_pose(np.array([-1.2,0.,1.5]),(1.,0.,.65))
    cp,sp=math.cos(pan_rad),math.sin(pan_rad);ct,st=math.cos(tilt_rad),math.sin(tilt_rad)
    rz=np.array([[cp,-sp,0],[sp,cp,0],[0,0,1.]])
    up=np.array([[ct,0,-st],[0,1,0],[st,0,ct]])
    pose=base.copy();pose[:3,:3]=rz@up@base[:3,:3]
    return pose


class GimbalDemoSource:
    """Publisher-owned map plus nonblocking requests from the control service."""
    def __init__(self,retention_ms=30000):
        self.source_id=new_live_source_id()
        self.driver=SimulatedPanTilt();self.observation_map=RetainedObservationMap(retention_ms=retention_ms)
        self.clear_requested=Event();self.map_generation=self.observation_map.map_generation;self.server=None
    def request_clear(self):self.clear_requested.set()
    def service_clear(self):
        if self.clear_requested.is_set():
            self.clear_requested.clear();self.observation_map.clear()
            self.map_generation=self.observation_map.map_generation
    def sample(self,sequence):
        self.service_clear()
        feedback=self.server.sample() if self.server is not None else self.driver.advance(time.monotonic_ns())
        cameras,frames=six_camera_batch(optical_pose(feedback.pan_rad,feedback.tilt_rad),feedback.timestamp_ns,sequence)
        return cameras,frames,feedback.tracked
    def observe_packet(self,packet):self.map_generation=packet.map_generation


def make_gimbal_demo(*,geometry='prepared',retention_ms=30000):
    """Deterministic final snapshot after a measured virtual pan sweep, never hardware."""
    if geometry not in ('prepared','unprepared'):raise ValueError('gimbal RGB demo requires prepared or unprepared geometry')
    cache=RetainedObservationMap(retention_ms=retention_ms);driver=SimulatedPanTilt(start_ns=1000000000)
    packet=stats=None
    # Targets are fed to a real simulated backend. Render from feedback, never target angles.
    for index,target in enumerate((0.,.25,.5,.25,0.),1):
        start=1000000000+(index-1)*300000000
        driver.target(target,0.,start);feedback=driver.advance(start+200000000)
        cameras,frames=six_camera_batch(optical_pose(feedback.pan_rad,feedback.tilt_rad),feedback.timestamp_ns,index)
        packet,stats=build_rgb_batch(frames,cameras,geometry=geometry,source_id=STATIC_SOURCE_ID,world_epoch=1,sequence=index,calibration_id=1,
            produced_ns=feedback.timestamp_ns+1000000,rig_mode='posed',observer_pose=OBSERVER,synthetic=True,
            observation_map=cache,retention_enabled=True,retention_ms=retention_ms)
    return packet,dict(stats,source='synthetic six-camera RGB-D with measured simulated gimbal sweep')
