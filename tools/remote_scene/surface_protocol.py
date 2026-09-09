"""Bounded RSCN v2 calibrated-image and projectively textured surface packets."""
from dataclasses import dataclass
import math
import struct
import zlib
import numpy as np
from .protocol import HEADER, integer, validate_common

EXTENSION=struct.Struct('<8I')
VIEW_HEADER=struct.Struct('<I4HI33fIQ')
MAX_VERTICES=16384
MAX_INDICES=98304
MAX_PIXELS=384000
MAX_DIMENSION=2048
MAX_RAW_V2=160+24*MAX_VERTICES+2*MAX_INDICES+MAX_PIXELS
PREPARED,UNPREPARED=1,2


def readonly(array,dtype=None):
    result=np.array(array,dtype=dtype,copy=True,order='C')
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class SurfaceView:
    camera_id:int
    depth_intrinsics:tuple
    intensity_intrinsics:tuple
    depth_units_m:float
    world_from_depth:np.ndarray
    intensity_from_depth:np.ndarray
    capture_ns:int
    depth:np.ndarray
    intensity:np.ndarray


@dataclass(frozen=True)
class SurfacePacket:
    source_id:int
    world_epoch:int
    sequence:int
    calibration_id:int
    capture_start_ns:int
    capture_end_ns:int
    produced_ns:int
    expected_mask:int
    contributing_mask:int
    flags:int
    max_capture_skew_ms:int
    observer_pose:tuple
    representation:int
    vertices:np.ndarray
    indices:np.ndarray
    atlas:np.ndarray
    views:tuple=()

    @property
    def identity(self):return self.source_id,self.world_epoch,self.calibration_id,self.representation,0
    @property
    def voxel_size_m(self):return 0.
    @property
    def geometry_summary(self):
        return dict(representation=self.representation,vertices=len(self.vertices),triangles=len(self.indices)//3,
                    atlas_width=self.atlas.shape[1],atlas_height=self.atlas.shape[0],views=len(self.views))
    @property
    def has_geometry(self):return bool(len(self.indices))


def _intrinsics(value,name):
    array=np.asarray(value,dtype=np.float64)
    if array.shape!=(4,) or not np.isfinite(array).all() or array[0]<=0 or array[1]<=0:
        raise ValueError(f'invalid {name} intrinsics')
    if np.any(np.abs(array)>np.finfo(np.float32).max):raise ValueError(f'invalid {name} float32 intrinsics')
    result=array.astype(np.float32).astype(np.float64)
    if result[0]<=0 or result[1]<=0:raise ValueError(f'invalid {name} serialized intrinsics')
    return result


def _transform(value):
    array=np.asarray(value,dtype=np.float64)
    if array.shape!=(3,4) or not np.isfinite(array).all() or np.any(np.abs(array)>np.finfo(np.float32).max):
        raise ValueError('invalid surface transform')
    array=array.astype(np.float32).astype(np.float64)
    rotation=array[:,:3]
    if (np.any(np.abs(array[:,3])>100) or np.any(np.abs(rotation.T@rotation-np.eye(3))>1e-3)
            or np.any(np.abs(rotation@rotation.T-np.eye(3))>1e-3) or abs(np.linalg.det(rotation)-1)>1e-3):
        raise ValueError('surface transform must be bounded right-handed rigid')
    return array


def validate_view(view,start,end):
    if not isinstance(view,SurfaceView):raise ValueError('invalid surface view')
    integer(view.camera_id,0,4,'camera ID');integer(view.capture_ns,start,end,'view capture')
    for array,dtype,maxw,maxh,name in ((view.depth,np.dtype('<u2'),64,48,'depth'),(view.intensity,np.dtype('u1'),320,240,'intensity')):
        if not isinstance(array,np.ndarray) or array.dtype!=dtype or array.ndim!=2 or not 2<=array.shape[1]<=maxw or not 2<=array.shape[0]<=maxh:
            raise ValueError(f'invalid bounded {name} image')
    _intrinsics(view.depth_intrinsics,'depth');_intrinsics(view.intensity_intrinsics,'intensity')
    if not isinstance(view.depth_units_m,(int,float,np.floating)) or not math.isfinite(view.depth_units_m) or not 0<float(np.float32(view.depth_units_m))<=1:
        raise ValueError('invalid depth units')
    _transform(view.world_from_depth);_transform(view.intensity_from_depth)


def validate_mesh(vertices,indices,atlas):
    if not isinstance(vertices,np.ndarray) or vertices.dtype!=np.dtype('<f4') or vertices.ndim!=2 or vertices.shape[1]!=6 or len(vertices)>MAX_VERTICES:
        raise ValueError('invalid surface vertex array')
    if not isinstance(indices,np.ndarray) or indices.dtype!=np.dtype('<u2') or indices.ndim!=1 or len(indices)>MAX_INDICES or len(indices)%3:
        raise ValueError('invalid surface index array')
    if not isinstance(atlas,np.ndarray) or atlas.dtype!=np.dtype('u1') or atlas.ndim!=2 or max(atlas.shape)>MAX_DIMENSION or atlas.size>MAX_PIXELS:
        raise ValueError('invalid surface atlas')
    if not len(vertices) and not len(indices) and atlas.shape==(0,0):return
    if not len(vertices) or not len(indices) or min(atlas.shape)==0:raise ValueError('inconsistent empty surface')
    if not np.isfinite(vertices).all() or np.any(np.abs(vertices[:,:3])>100):raise ValueError('invalid surface coordinates')
    q=vertices[:,5].astype(np.float64)
    # Bound the serialized float32 Q so boundary values survive a round trip.
    if np.any(q<float(np.float32(1e-6))) or np.any(q>1024):raise ValueError('invalid surface projective Q')
    uv=vertices[:,3:5].astype(np.float64)/q[:,None]
    if np.any(uv<0) or np.any(uv>1):raise ValueError('invalid normalized surface UV')
    if np.any(indices>=len(vertices)):raise ValueError('surface index out of range')
    tris=indices.reshape(-1,3)
    if np.any(tris[:,0]==tris[:,1]) or np.any(tris[:,0]==tris[:,2]) or np.any(tris[:,1]==tris[:,2]):raise ValueError('repeated triangle index')
    p=vertices[tris,:3].astype(np.float64)
    area=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0])
    if np.any(np.sum(area*area,axis=1)<=1e-12):raise ValueError('degenerate surface triangle')


def validate_surface(packet):
    if not isinstance(packet,SurfacePacket):raise ValueError('invalid surface packet type')
    validate_common(packet)
    integer(packet.representation,PREPARED,UNPREPARED,'surface representation')
    validate_mesh(packet.vertices,packet.indices,packet.atlas)
    if packet.representation==PREPARED:
        if packet.views:raise ValueError('prepared packet contains image views')
    else:
        if not isinstance(packet.views,tuple) or len(packet.views)>5:raise ValueError('invalid view array')
        ids=[]
        for view in packet.views:
            validate_view(view,packet.capture_start_ns,packet.capture_end_ns);ids.append(view.camera_id)
        if ids!=sorted(set(ids)) or sum(1<<i for i in ids)!=packet.contributing_mask:raise ValueError('view IDs disagree with camera mask')
    if not packet.contributing_mask and (len(packet.vertices) or packet.views):raise ValueError('no-camera surface contains data')


def pack_surface(packet):
    validate_surface(packet)
    if packet.representation==PREPARED:
        count,index_count=len(packet.vertices),len(packet.indices);height,width=packet.atlas.shape
        payload=packet.vertices.tobytes(order='C')+packet.indices.tobytes(order='C')+packet.atlas.tobytes(order='C')
    else:
        count=index_count=width=height=0
        pieces=[]
        for view in packet.views:
            dh,dw=view.depth.shape;ih,iw=view.intensity.shape
            pieces.append(VIEW_HEADER.pack(view.camera_id,dw,dh,iw,ih,0,*view.depth_intrinsics,*view.intensity_intrinsics,
                view.depth_units_m,*np.asarray(view.world_from_depth).reshape(-1),*np.asarray(view.intensity_from_depth).reshape(-1),0,view.capture_ns))
            pieces.extend((view.depth.tobytes(order='C'),view.intensity.tobytes(order='C')))
        payload=b''.join(pieces)
    header=HEADER.pack(b'RSCN',2,160,packet.source_id,packet.world_epoch,packet.sequence,packet.calibration_id,
        packet.capture_start_ns,packet.capture_end_ns,packet.produced_ns,0,packet.expected_mask,packet.contributing_mask,
        packet.flags,0.,packet.max_capture_skew_ms,zlib.crc32(payload),0,*packet.observer_pose,0)
    return header+EXTENSION.pack(packet.representation,count,index_count,width,height,packet.contributing_mask.bit_count(),len(payload),0)+payload


def unpack_surface(raw):
    if not 160<=len(raw)<=MAX_RAW_V2:raise ValueError('invalid surface packet length')
    h=HEADER.unpack_from(raw);ext=EXTENSION.unpack_from(raw,128)
    representation,nv,ni,width,height,nviews,payload_bytes,reserved=ext
    if h[:3]!=(b'RSCN',2,160) or h[10] or struct.unpack_from('<I',raw,80)[0] or h[17] or h[25] or reserved:
        raise ValueError('invalid surface header or reserved field')
    if representation not in (PREPARED,UNPREPARED) or payload_bytes!=len(raw)-160 or nviews!=h[12].bit_count() or nviews>5:
        raise ValueError('invalid surface extension')
    if zlib.crc32(raw[160:])!=h[16]:raise ValueError('surface payload CRC mismatch')
    views=()
    if representation==PREPARED:
        if nv>MAX_VERTICES or ni>MAX_INDICES or ni%3 or width>MAX_DIMENSION or height>MAX_DIMENSION or width*height>MAX_PIXELS:
            raise ValueError('surface counts exceed budget')
        if 24*nv+2*ni+width*height!=payload_bytes:raise ValueError('surface payload length mismatch')
        # Retain immutable owned bytes even when the caller supplied mutable storage.
        payload=bytes(raw[160:]);vertices=np.frombuffer(payload,dtype='<f4',count=6*nv).reshape(nv,6)
        indices=np.frombuffer(payload,dtype='<u2',count=ni,offset=24*nv)
        atlas=np.frombuffer(payload,dtype='u1',count=width*height,offset=24*nv+2*ni).reshape(height,width)
    else:
        if nv or ni or width or height:raise ValueError('unprepared packet declares mesh fields')
        offset=160;descriptors=[]
        for _ in range(nviews):
            if offset+160>len(raw):raise ValueError('truncated view descriptor')
            v=VIEW_HEADER.unpack_from(raw,offset);offset+=160
            camera,dw,dh,iw,ih,vreserved=v[:6]
            if vreserved or v[39] or not (0<=camera<=4 and 2<=dw<=64 and 2<=dh<=48 and 2<=iw<=320 and 2<=ih<=240):
                raise ValueError('invalid view descriptor dimensions/reserved')
            image_end=offset+2*dw*dh+iw*ih
            if image_end>len(raw):raise ValueError('truncated view images')
            descriptors.append((v,offset));offset=image_end
        if offset!=len(raw):raise ValueError('unprepared payload trailing data')
        items=[]
        for v,offset in descriptors:
            camera,dw,dh,iw,ih=v[:5];payload=bytes(raw[offset:offset+2*dw*dh+iw*ih])
            items.append(SurfaceView(camera,tuple(v[6:10]),tuple(v[10:14]),v[14],readonly(np.array(v[15:27]).reshape(3,4),'<f4'),
                readonly(np.array(v[27:39]).reshape(3,4),'<f4'),v[40],np.frombuffer(payload,dtype='<u2',count=dw*dh).reshape(dh,dw),
                np.frombuffer(payload,dtype='u1',count=iw*ih,offset=2*dw*dh).reshape(ih,iw)))
        views=tuple(items);vertices=readonly(np.empty((0,6)),'<f4');indices=readonly([], '<u2');atlas=readonly(np.empty((0,0)),'u1')
    packet=SurfacePacket(*h[3:10],h[11],h[12],h[13],h[15],tuple(h[18:25]),representation,vertices,indices,atlas,views)
    validate_surface(packet)
    return packet
