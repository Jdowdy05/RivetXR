"""Measured pan/tilt pose adapter and an explicitly simulation-only backend.

Robot axes: X forward,Y left,Z up. Positive pan is Rz(pan); positive tilt
looks up, Ry(-tilt). Sensor optical axes/offsets belong in rig_from_depth.
No SDK, serial, motor, network or headset operations exist in this module.
"""
from dataclasses import dataclass
import math
from threading import RLock
import time
from typing import Protocol
import numpy as np
from .calibration import rigid
from .pose_history import PoseHistory,PanTiltHistory,timestamp
from .protocol import integer


def _angle(value,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError(f'{name} must be finite radians')
    return float(value)


def _rz(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1.]])


def _up_tilt(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,0,-s,0],[0,1,0,0],[s,0,c,0],[0,0,0,1.]])


@dataclass(frozen=True)
class MotionBounds:
    translation_m:float
    rotation_rad:float


class RigPoseProvider:
    """Callable Capture adapter using measured localization and encoder histories.

    Fixed cameras' rig is the robot body. The gimbal camera's rig is the final
    tilt frame; its CameraCalibration.rig_from_depth supplies the optical offset.
    Returned matrices are detached. No commanded or current-head pose is used.
    """
    def __init__(self,robot:PoseHistory,angles:PanTiltHistory,*,robot_from_mount,pan_from_tilt,gimbal_camera_id=5):
        self.robot=robot;self.angles=angles
        self.robot_from_mount=rigid(robot_from_mount,'robot_from_mount')
        self.pan_from_tilt=rigid(pan_from_tilt,'pan_from_tilt')
        self.gimbal_camera_id=integer(gimbal_camera_id,0,5,'gimbal camera ID')

    def __call__(self,camera_id,exposure_ns):
        integer(camera_id,0,5,'camera ID')
        world=self.robot.at(exposure_ns)
        if camera_id==self.gimbal_camera_id:
            pan,tilt=self.angles.at(exposure_ns)
            world=world@self.robot_from_mount@_rz(pan)@self.pan_from_tilt@_up_tilt(tilt)
        return rigid(world,'measured world_from_rig')

    def world_from_depth(self,camera_id,exposure_ns,rig_from_depth):
        return rigid(self(camera_id,exposure_ns)@rigid(rig_from_depth),'measured world_from_depth')

    def image_from_depth(self,camera_id,depth_ns,image_ns,rig_from_depth,calibration_image_from_depth):
        depth=self.world_from_depth(camera_id,depth_ns,rig_from_depth)
        image_time_depth=self.world_from_depth(camera_id,image_ns,rig_from_depth)
        return rigid(rigid(calibration_image_from_depth)@np.linalg.inv(image_time_depth)@depth,'cross-exposure image_from_depth')

    def motion_bounds(self,camera_id,start_ns,end_ns,rig_from_depth):
        """Conservative camera path-length bounds including intermediate motion.

        Bounds apply to the bracketed piecewise measured interpolation, not to
        arbitrary unobserved high-frequency motion. Lever-arm terms prevent a
        returning pan/tilt sweep from masquerading as stationary end poses.
        """
        integer(camera_id,0,5,'camera ID');sensor=rigid(rig_from_depth)
        translation,rotation=self.robot.motion_bounds(start_ns,end_ns)
        lever=float(np.linalg.norm(sensor[:3,3]))
        if camera_id!=self.gimbal_camera_id:return MotionBounds(translation+lever*rotation,rotation)
        pan,tilt=self.angles.motion_bounds(start_ns,end_ns)
        pivot=float(np.linalg.norm(self.pan_from_tilt[:3,3]));mount=float(np.linalg.norm(self.robot_from_mount[:3,3]))
        return MotionBounds(translation+(mount+pivot+lever)*rotation+(pivot+lever)*pan+lever*tilt,rotation+pan+tilt)


@dataclass(frozen=True)
class PanTiltLimits:
    # Simulation defaults, not the limits of an unidentified physical gimbal.
    pan_min_rad:float=-math.pi
    pan_max_rad:float=math.pi
    tilt_min_rad:float=-math.pi/2
    tilt_max_rad:float=math.pi/2
    max_rate_rad_s:float=1.
    command_timeout_ns:int=250_000_000

    def __post_init__(self):
        for name in ('pan_min_rad','pan_max_rad','tilt_min_rad','tilt_max_rad','max_rate_rad_s'):_angle(getattr(self,name),name)
        if not -1000<=self.pan_min_rad<self.pan_max_rad<=1000 or not -1000<=self.tilt_min_rad<self.tilt_max_rad<=1000:
            raise ValueError('invalid simulated angle limits')
        if not 0<self.max_rate_rad_s<=20:raise ValueError('invalid simulated speed limit')
        integer(self.command_timeout_ns,1_000_000,1_000_000_000,'simulated command timeout')


@dataclass(frozen=True)
class GimbalFeedback:
    timestamp_ns:int
    pan_rad:float
    tilt_rad:float
    tracked:bool
    command_stale:bool
    simulation_only:bool=True


class PanTiltBackend(Protocol):
    """Adapter contract; this package ships no physical implementation."""
    simulation_only:bool
    def target(self,pan_rad:float,tilt_rad:float,timestamp_ns:int)->GimbalFeedback: ...
    def advance(self,timestamp_ns:int)->GimbalFeedback: ...
    def hold(self,timestamp_ns:int)->GimbalFeedback: ...
    def feedback(self)->GimbalFeedback: ...


class SimulatedPanTilt:
    simulation_only=True
    def __init__(self,*,limits=None,start_ns=None):
        self.limits=PanTiltLimits() if limits is None else limits
        if not isinstance(self.limits,PanTiltLimits):raise ValueError('validated simulated limits required')
        if not self.limits.pan_min_rad<=0<=self.limits.pan_max_rad or not self.limits.tilt_min_rad<=0<=self.limits.tilt_max_rad:
            raise ValueError('simulation zero seed must fit limits')
        self._time=timestamp(time.monotonic_ns() if start_ns is None else start_ns)
        self._pan=self._tilt=0.;self._target=(0.,0.);self._command=None;self._tracked=True
        self._lock=RLock();self.history=PanTiltHistory()
        self.history.append(self._time,0.,0.)

    def _feedback(self):
        stale=self._command is None or self._time-self._command>self.limits.command_timeout_ns
        return GimbalFeedback(self._time,self._pan,self._tilt,self._tracked,stale)

    def feedback(self):
        with self._lock:return self._feedback()

    def _advance(self,stamp,publish=True):
        timestamp(stamp)
        if stamp<self._time:raise ValueError('simulation time reversed')
        if stamp==self._time:return self._feedback()
        if self._tracked and self._command is not None:
            end=min(stamp,self._command+self.limits.command_timeout_ns)
            dt=max(0,end-self._time)/1e9
            values=[]
            for actual,target in zip((self._pan,self._tilt),self._target):
                speed=self.limits.max_rate_rad_s
                values.append(actual+max(-speed*dt,min(speed*dt,target-actual)))
            self._pan,self._tilt=values
        self._time=stamp
        if publish:self.history.append(stamp,self._pan,self._tilt,tracked=self._tracked)
        return self._feedback()

    def advance(self,timestamp_ns):
        with self._lock:return self._advance(timestamp_ns)

    def target(self,pan_rad,tilt_rad,timestamp_ns):
        pan=_angle(pan_rad,'pan');tilt=_angle(tilt_rad,'tilt');timestamp(timestamp_ns)
        with self._lock:
            if not self._tracked:raise ValueError('cannot target a tracking-invalid simulated gimbal')
            if not self.limits.pan_min_rad<=pan<=self.limits.pan_max_rad or not self.limits.tilt_min_rad<=tilt<=self.limits.tilt_max_rad:
                raise ValueError('simulated target exceeds angle limits')
            self._advance(timestamp_ns);self._target=(pan,tilt);self._command=timestamp_ns
            return self._feedback()

    set_target=target

    def hold(self,timestamp_ns):
        with self._lock:
            self._advance(timestamp_ns);self._target=(self._pan,self._tilt);self._command=None
            return self._feedback()

    def set_tracking(self,tracked,timestamp_ns):
        if type(tracked) is not bool:raise ValueError('tracked must be boolean')
        timestamp(timestamp_ns)
        with self._lock:
            if tracked!=self._tracked and timestamp_ns<=self._time:
                raise ValueError('tracking changes require a newer measured timestamp')
            if tracked==self._tracked:return self._advance(timestamp_ns)
            self._advance(timestamp_ns,publish=False);self._tracked=tracked
            self._target=(self._pan,self._tilt);self._command=None
            self.history.append(timestamp_ns,self._pan,self._tilt,tracked=tracked)
            return self._feedback()
