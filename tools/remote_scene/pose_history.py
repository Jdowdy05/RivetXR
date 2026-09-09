"""Bounded measured exposure poses; no extrapolation, prediction or file writes.

All timestamps use the same producer_monotonic_ns clock as camera exposures.
Robot rotations interpolate by shortest quaternion SLERP. Pan/tilt telemetry
must already be unwrapped radians; no encoder-wrap convention is guessed.
"""
from bisect import bisect_left
from dataclasses import dataclass
import json
import math
from pathlib import Path
from threading import RLock
import numpy as np
from .calibration import rigid
from .protocol import integer

MAX_SAMPLES=8192
MAX_TRACE_BYTES=8*1024*1024


class PoseUnavailable(ValueError):
    """No trustworthy measured pose covers the requested exposure/window."""


def timestamp(value):return integer(value,1,2**64-1,'pose timestamp')


def _quaternion(matrix):
    r=np.asarray(matrix,dtype=np.float64)[:3,:3]
    trace=float(np.trace(r))
    if trace>0:
        s=2*math.sqrt(1+trace)
        q=np.array([(r[2,1]-r[1,2])/s,(r[0,2]-r[2,0])/s,(r[1,0]-r[0,1])/s,s/4])
    else:
        i=int(np.argmax(np.diag(r)));j=(i+1)%3;k=(i+2)%3
        s=2*math.sqrt(max(0.,1+float(r[i,i]-r[j,j]-r[k,k])))
        if s<=1e-12:raise ValueError('invalid measured rotation')
        q=np.empty(4);q[i]=s/4;q[j]=(r[j,i]+r[i,j])/s;q[k]=(r[k,i]+r[i,k])/s;q[3]=(r[k,j]-r[j,k])/s
    return q/np.linalg.norm(q)


def _rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def _slerp(a,b,fraction):
    dot=float(np.dot(a,b))
    if dot<0:b=-b;dot=-dot
    dot=min(1.,max(-1.,dot))
    if dot>.9995:q=a+(b-a)*fraction
    else:
        angle=math.acos(dot);q=(math.sin((1-fraction)*angle)*a+math.sin(fraction*angle)*b)/math.sin(angle)
    return q/np.linalg.norm(q)


class _History:
    def __init__(self,capacity=512,max_gap_ns=50_000_000):
        self.capacity=integer(capacity,2,MAX_SAMPLES,'history capacity')
        self.max_gap_ns=integer(max_gap_ns,1,1_000_000_000,'interpolation gap')
        self._times=[];self._values=[];self._lock=RLock()

    def _append(self,stamp,value,tracked):
        timestamp(stamp)
        if type(tracked) is not bool:raise ValueError('tracked must be boolean')
        if tracked and value is None:raise ValueError('tracked sample requires measured state')
        with self._lock:
            if self._times and stamp<=self._times[-1]:raise ValueError('pose timestamps must strictly increase')
            self._times.append(stamp);self._values.append(value if tracked else None)
            if len(self._times)>self.capacity:
                self._times.pop(0);self._values.pop(0)

    def _data(self):
        with self._lock:return tuple(self._times),tuple(self._values)

    def _at(self,stamp,times,values):
        timestamp(stamp);index=bisect_left(times,stamp)
        if index<len(times) and times[index]==stamp:
            if values[index] is None:raise PoseUnavailable('tracking invalid at exposure')
            return values[index]
        if index==0 or index==len(times):raise PoseUnavailable('exposure is not bracketed by measured poses')
        a,b=values[index-1],values[index]
        if a is None or b is None:raise PoseUnavailable('tracking loss crosses exposure interval')
        gap=times[index]-times[index-1]
        if gap>self.max_gap_ns:raise PoseUnavailable('measured pose gap exceeds interpolation bound')
        return self._interpolate(a,b,(stamp-times[index-1])/gap)

    def _window(self,start,end):
        timestamp(start);timestamp(end)
        if end<start:raise ValueError('pose window runs backwards')
        times,values=self._data()
        knots=[start,*[t for t in times if start<t<end]]
        if end!=start:knots.append(end)
        if any(b-a>self.max_gap_ns for a,b in zip(knots,knots[1:])):
            raise PoseUnavailable('pose window contains an excessive sampling gap')
        return [self._at(t,times,values) for t in knots]


class PoseHistory(_History):
    """Thread-safe, bounded world_from_robot matrices from measured localization."""
    def append(self,timestamp_ns,world_from_robot=None,*,tracked=True):
        value=None if world_from_robot is None else tuple(map(float,rigid(world_from_robot).ravel()))
        self._append(timestamp_ns,value,tracked)

    @staticmethod
    def _interpolate(a,b,fraction):
        a=np.asarray(a).reshape(4,4);b=np.asarray(b).reshape(4,4);out=np.eye(4)
        out[:3,3]=a[:3,3]+(b[:3,3]-a[:3,3])*fraction
        out[:3,:3]=_rotation(_slerp(_quaternion(a),_quaternion(b),fraction))
        return tuple(map(float,out.ravel()))

    def at(self,timestamp_ns):
        return rigid(self._at(timestamp_ns,*self._data()),'interpolated measured pose')

    def motion_bounds(self,start_ns,end_ns):
        """Translation/rotation path lengths of the measured interpolation."""
        values=[np.asarray(value).reshape(4,4) for value in self._window(start_ns,end_ns)]
        translation=rotation=0.
        for a,b in zip(values,values[1:]):
            translation+=float(np.linalg.norm(b[:3,3]-a[:3,3]))
            dot=min(1.,abs(float(np.dot(_quaternion(a),_quaternion(b)))))
            rotation+=2*math.acos(dot)
        return translation,rotation


class PanTiltHistory(_History):
    """Measured, unwrapped radians; an invalid marker interrupts interpolation."""
    def append(self,timestamp_ns,pan_rad=None,tilt_rad=None,*,tracked=True):
        value=None
        if pan_rad is not None or tilt_rad is not None:
            pair=(pan_rad,tilt_rad)
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>1000 for v in pair):
                raise ValueError('measured pan/tilt require finite unwrapped radians')
            value=tuple(map(float,pair))
        self._append(timestamp_ns,value,tracked)

    @staticmethod
    def _interpolate(a,b,fraction):return tuple(x+(y-x)*fraction for x,y in zip(a,b))

    def at(self,timestamp_ns):return self._at(timestamp_ns,*self._data())

    def motion_bounds(self,start_ns,end_ns):
        values=self._window(start_ns,end_ns)
        return tuple(sum(abs(b[axis]-a[axis]) for a,b in zip(values,values[1:])) for axis in (0,1))


@dataclass(frozen=True)
class PoseTrace:
    robot:PoseHistory
    gimbal:PanTiltHistory


def read_pose_trace(path,*,max_gap_ns=50_000_000):
    """Read bounded measured telemetry. Never write/retime it or fabricate poses.

    JSON: version1, clock producer_monotonic_ns, robot/gimbal arrays. Robot rows
    contain timestamp_ns,tracked,world_from_robot (16 row-major values or null).
    Gimbal rows contain timestamp_ns,tracked,pan_rad,tilt_rad (null when lost).
    """
    def unique(items):
        result={}
        for key,value in items:
            if key in result:raise ValueError('duplicate pose trace field')
            result[key]=value
        return result
    def fields(value,names):
        if not isinstance(value,dict) or set(value)!=set(names):raise ValueError('unknown/missing pose trace fields')
    with Path(path).open('rb') as stream:raw=stream.read(MAX_TRACE_BYTES+1)
    if len(raw)>MAX_TRACE_BYTES:raise ValueError('pose trace exceeds byte bound')
    data=json.loads(raw.decode('utf-8'),object_pairs_hook=unique,
        parse_constant=lambda _:(_ for _ in ()).throw(ValueError('nonfinite pose trace constant')))
    fields(data,('version','clock','robot','gimbal'))
    if type(data['version']) is not int or data['version']!=1 or data['clock']!='producer_monotonic_ns':
        raise ValueError('unsupported pose trace version/clock')
    for name in ('robot','gimbal'):
        if not isinstance(data[name],list) or not 1<=len(data[name])<=MAX_SAMPLES:raise ValueError('pose trace sample count outside bounds')
    robot=PoseHistory(max(2,len(data['robot'])),max_gap_ns);gimbal=PanTiltHistory(max(2,len(data['gimbal'])),max_gap_ns)
    for row in data['robot']:
        fields(row,('timestamp_ns','tracked','world_from_robot'))
        robot.append(row['timestamp_ns'],row['world_from_robot'],tracked=row['tracked'])
    for row in data['gimbal']:
        fields(row,('timestamp_ns','tracked','pan_rad','tilt_rad'))
        gimbal.append(row['timestamp_ns'],row['pan_rad'],row['tilt_rad'],tracked=row['tracked'])
    return PoseTrace(robot,gimbal)
