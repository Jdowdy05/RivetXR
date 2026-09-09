"""Rectified depth/Y8 contracts. Arbitrary distortion is explicitly unsupported."""
from dataclasses import dataclass
import math
import numpy as np
from .protocol import integer

MAX_WIDTH,MAX_HEIGHT=1920,1080


def rigid(value,name='transform'):
    matrix=np.asarray(value,dtype=np.float64)
    if matrix.size==16:matrix=matrix.reshape(4,4)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3],[0,0,0,1],atol=1e-9,rtol=0):raise ValueError(f'invalid {name}')
    rotation=matrix[:3,:3]
    if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-4,rtol=0) or abs(np.linalg.det(rotation)-1)>1e-4 or np.any(np.abs(matrix[:3,3])>100):
        raise ValueError(f'{name} must be a bounded right-handed rigid transform')
    matrix=matrix.copy();matrix.setflags(write=False);return matrix


@dataclass(frozen=True)
class Intrinsics:
    width:int
    height:int
    fx:float
    fy:float
    ppx:float
    ppy:float
    distortion:str='none'
    coeffs:tuple=(0.,0.,0.,0.,0.)
    def validate(self):
        integer(self.width,1,MAX_WIDTH,'width');integer(self.height,1,MAX_HEIGHT,'height')
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (self.fx,self.fy,self.ppx,self.ppy)):raise ValueError('invalid intrinsics')
        if self.fx<=0 or self.fy<=0 or abs(self.ppx)>MAX_WIDTH or abs(self.ppy)>MAX_HEIGHT:raise ValueError('invalid focal/principal point')
        if self.distortion!='none' or len(self.coeffs)!=5 or any(v!=0 or type(v) not in (int,float) for v in self.coeffs):
            raise ValueError('v1 requires queried rectified profiles with no distortion')


@dataclass(frozen=True)
class CameraCalibration:
    camera_id:int
    depth:Intrinsics
    intensity:Intrinsics
    depth_units_m:float
    rig_from_depth:np.ndarray
    intensity_from_depth:np.ndarray
    image_format:str='Y8'
    def __post_init__(self):
        integer(self.camera_id,0,5,'camera ID');self.depth.validate();self.intensity.validate()
        if type(self.depth_units_m) not in (int,float) or not math.isfinite(self.depth_units_m) or not 0<self.depth_units_m<=1:raise ValueError('invalid depth units')
        if self.image_format not in ('Y8','RGB8'):raise ValueError('image format must be Y8 or RGB8')
        object.__setattr__(self,'rig_from_depth',rigid(self.rig_from_depth,'rig_from_depth'))
        object.__setattr__(self,'intensity_from_depth',rigid(self.intensity_from_depth,'intensity_from_depth'))


@dataclass(frozen=True)
class DepthFrame:
    camera_id:int
    sequence:int
    capture_ns:int
    depth:np.ndarray
    intensity:np.ndarray
    world_from_rig:np.ndarray|None=None
    image_capture_ns:int|None=None
    image_from_depth:np.ndarray|None=None
    image_usable:bool=True
    @property
    def image_timestamp_ns(self):return self.capture_ns if self.image_capture_ns is None else self.image_capture_ns
    def image_pose(self,calibration):
        return calibration.intensity_from_depth if self.image_from_depth is None else rigid(self.image_from_depth,'exposure image_from_depth')
    def validate(self,calibration,rig_mode):
        if self.camera_id!=calibration.camera_id:raise ValueError('frame/calibration camera mismatch')
        integer(self.sequence,1,2**64-1,'frame sequence');integer(self.capture_ns,1,2**64-1,'capture time')
        if not isinstance(self.depth,np.ndarray) or self.depth.dtype!=np.dtype('<u2') or self.depth.shape!=(calibration.depth.height,calibration.depth.width):raise ValueError('depth must be little-endian Z16 matching its profile')
        shape=(calibration.intensity.height,calibration.intensity.width)+((3,) if calibration.image_format=='RGB8' else ())
        if not isinstance(self.intensity,np.ndarray) or self.intensity.dtype!=np.dtype('u1') or self.intensity.shape!=shape:raise ValueError('image must match declared Y8/RGB8 profile')
        integer(self.image_timestamp_ns,1,2**64-1,'image capture time')
        if type(self.image_usable) is not bool:raise ValueError('image usability must be boolean')
        self.image_pose(calibration)
        if rig_mode=='stationary':
            if self.world_from_rig is not None and not np.array_equal(rigid(self.world_from_rig),np.eye(4)):raise ValueError('stationary rig must use its fixed map frame')
        elif rig_mode=='posed':
            if self.world_from_rig is None:raise ValueError('moving rig requires world pose at exposure time')
            rigid(self.world_from_rig,'capture-time world_from_rig')
        else:raise ValueError('explicit rig_mode stationary or posed is required')


def transform(points,matrix):return points@matrix[:3,:3].T+matrix[:3,3]


def require_legacy(frame,calibration):
    if calibration.camera_id>4 or calibration.image_format!='Y8':raise ValueError('legacy points/surfaces require five Y8 cameras')
    if (frame.image_timestamp_ns!=frame.capture_ns or not frame.image_usable
            or not np.array_equal(frame.image_pose(calibration),calibration.intensity_from_depth)):
        raise ValueError('separate image exposure or unusable images require RGB v3')


def project_frame(frame,calibration,rig_mode):
    require_legacy(frame,calibration)
    frame.validate(calibration,rig_mode)
    y,x=np.nonzero(frame.depth)
    z=frame.depth[y,x].astype(np.float64)*calibration.depth_units_m
    d=calibration.depth
    points=np.column_stack(((x-d.ppx)*z/d.fx,(y-d.ppy)*z/d.fy,z))
    ir=transform(points,calibration.intensity_from_depth);i=calibration.intensity
    valid=(ir[:,2]>0)&np.isfinite(ir).all(axis=1)
    selected=np.flatnonzero(valid);uv=ir[valid,:2]/ir[valid,2,None]
    uv=uv*np.array([i.fx,i.fy])+np.array([i.ppx,i.ppy])
    visible=np.isfinite(uv).all(axis=1)&(uv[:,0]>=-.5)&(uv[:,0]<i.width-.5)&(uv[:,1]>=-.5)&(uv[:,1]<i.height-.5)
    selected=selected[visible];pixels=np.floor(uv[visible]+.5).astype(np.int64)
    # A z-buffer rejects obviously occluded intensity correspondences. It only
    # uses this view's observed samples and does not hallucinate hidden texture.
    flat=pixels[:,1]*i.width+pixels[:,0]
    nearest=np.full(i.width*i.height,np.inf);np.minimum.at(nearest,flat,ir[selected,2])
    visible=ir[selected,2]<=nearest[flat]+.005
    selected=selected[visible];pixels=pixels[visible]
    gray=frame.intensity[pixels[:,1],pixels[:,0]]
    world=transform(points[selected],calibration.rig_from_depth)
    if rig_mode=='posed':world=transform(world,rigid(frame.world_from_rig))
    keep=np.isfinite(world).all(axis=1)&np.all(np.abs(world)<=100,axis=1)
    return world[keep],gray[keep]
