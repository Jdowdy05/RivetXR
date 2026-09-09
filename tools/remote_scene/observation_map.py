"""Bounded retained depth/image observations; not TSDF, SLAM or hidden completion."""
from dataclasses import replace
import copy
import math
import numpy as np
from .protocol import integer
from .rgb_protocol import readonly,_validate_raw,observation_stamp

TRANSLATION_NOVELTY_M=.10
ROTATION_NOVELTY_RAD=math.radians(10.)


def _distinct(a,b):
    x=np.asarray(a.world_from_depth,dtype='f4').astype(float);y=np.asarray(b.world_from_depth,dtype='f4').astype(float)
    distance=np.linalg.norm(x[:,3]-y[:,3]);angle=math.acos(float(np.clip((np.trace(x[:,:3].T@y[:,:3])-1)*.5,-1,1)))
    return distance>=TRANSLATION_NOVELTY_M or angle>=ROTATION_NOVELTY_RAD


def _project_depth(world,view):
    transform=np.asarray(view.world_from_depth,dtype='f4').astype(float)
    local=(world-transform[:,3])@transform[:,:3]
    k=np.asarray(view.depth_intrinsics,dtype='f4').astype(float)
    uv=np.divide(local[:,:2],local[:,2,None],out=np.full((len(world),2),np.nan),where=local[:,2,None]>0)*k[:2]+k[2:]
    h,w=view.depth.shape
    valid=np.isfinite(uv).all(axis=1)&(local[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<w-1)&(uv[:,1]>=0)&(uv[:,1]<h-1)
    pixels=np.floor(np.where(valid[:,None],uv,0)).astype(int)
    corners=np.stack((view.depth[pixels[:,1],pixels[:,0]],view.depth[pixels[:,1],pixels[:,0]+1],
                      view.depth[pixels[:,1]+1,pixels[:,0]],view.depth[pixels[:,1]+1,pixels[:,0]+1]),axis=1).astype(float)*float(np.float32(view.depth_units_m))
    valid &= np.all(corners>0,axis=1)&(np.ptp(corners,axis=1)<=.05+.02*np.min(corners,axis=1))
    return local,uv,pixels,corners,valid


def _carve(view,current):
    from .rgb_surface import grid_geometry
    world,_,_,_,_=grid_geometry(view);remove=np.zeros(len(world),dtype=bool)
    for fresh in current:
        local,uv,pixels,corners,valid=_project_depth(world,fresh)
        nearest=np.min(corners,axis=1)
        # Four valid, locally continuous depth samples must all show free space.
        remove |= valid&(local[:,2]<nearest-(.03+.02*np.minimum(local[:,2],nearest)))
    original=view.depth.reshape(-1);remove &= original!=0
    if not np.any(remove):return view,0
    depth=view.depth.copy();depth.reshape(-1)[remove]=0
    return replace(view,depth=readonly(depth)),int(np.count_nonzero(remove))


def _rgb_covers(world,view):
    from .rgb_surface import grid_geometry
    _,image,texvalid,triangles,keep=grid_geometry(view)
    local,uv,pixels,corners,valid=_project_depth(world,view);h,w=view.depth.shape
    fractional=uv-pixels
    which=(fractional[:,0]+fractional[:,1]>1).astype(int)
    triangle_index=2*(pixels[:,1]*(w-1)+pixels[:,0])+which
    chosen=triangles[triangle_index]
    valid &= keep[triangle_index]&np.all(texvalid[chosen],axis=1)
    # Confirm geometric coincidence at every queried corner/centroid.
    valid &= np.max(np.abs(corners-local[:,2,None]),axis=1)<=.02+.01*local[:,2]
    transform=np.asarray(view.intensity_from_depth,dtype='f4').astype(float)
    ip=local@transform[:,:3].T+transform[:,3];k=np.asarray(view.intensity_intrinsics,dtype='f4').astype(float)
    iu=np.divide(ip[:,:2],ip[:,2,None],out=np.full((len(ip),2),np.nan),where=ip[:,2,None]>0)*k[:2]+k[2:]
    ih,iw=view.intensity.shape[:2]
    valid &= (ip[:,2]>0)&np.isfinite(iu).all(axis=1)&(iu[:,0]>=0)&(iu[:,0]<=iw-1)&(iu[:,1]>=0)&(iu[:,1]<=ih-1)
    return valid


def _prioritize_color(current,retained):
    from .rgb_surface import grid_geometry
    colored=[v for v in retained if v.image_format==3 and v.flags&2]
    output=[];suppressed=0
    for view in current:
        if view.image_format!=1 or not colored:output.append(view);continue
        world,_,_,triangles,keep=grid_geometry(view);active=np.flatnonzero(keep)
        if not len(active):output.append(view);continue
        positions=world[triangles[active]];probes=np.concatenate((positions,np.mean(positions,axis=1)[:,None,:]),axis=1)
        covered=np.zeros(len(active),dtype=bool)
        for rgb in colored:
            # A disconnected receiver expires each observation independently.
            # Never remove newer gray geometry in favor of color that expires
            # first. Preserve both timestamps; overlap may leave seams/depth ties
            # until visual qualification, but cannot create an expiry-time hole.
            if observation_stamp(rgb)<observation_stamp(view):continue
            covered |= _rgb_covers(probes.reshape(-1,3),rgb).reshape(-1,4).all(axis=1)
        mask=np.unpackbits(np.frombuffer(view.triangle_keep,dtype='u1'),bitorder='little')[:len(triangles)].copy()
        mask[active[covered]]=0;suppressed+=int(np.count_nonzero(covered))
        output.append(replace(view,triangle_keep=np.packbits(mask,bitorder='little').tobytes()))
    return output,suppressed


class RetainedObservationMap:
    def __init__(self,retention_ms=30000,max_keyframes=4):
        integer(retention_ms,1,60000,'retention milliseconds');integer(max_keyframes,0,4,'retained keyframe budget')
        self.retention_ms=retention_ms;self.max_keyframes=max_keyframes;self.map_generation=1
        self._identity=None;self._tracking=True;self._produced_ns=0;self._anchors={};self._retained=[];self._latest={};self.last_stats={}
    def clear(self):
        if self.map_generation==2**64-1:raise ValueError('map generation exhausted')
        self.map_generation+=1;self._anchors={};self._retained=[];self._latest={};self._produced_ns=0
    def update(self,views,source_identity,produced_ns,*,tracking_valid=True,enabled=True):
        integer(produced_ns,1,2**64-1,'map production time')
        if len(source_identity)!=3:raise ValueError('invalid map source identity')
        for value in source_identity:integer(value,1,2**64-1,'map source identity')
        if type(tracking_valid) is not bool or type(enabled) is not bool:raise ValueError('invalid map state flags')
        if len(views)>6 or [v.camera_id for v in views]!=sorted(set(v.camera_id for v in views)):raise ValueError('invalid current map observations')
        for v in views:
            _validate_raw(v)
            if v.flags&1 or not 0<min(v.capture_ns,v.image_capture_ns)<=max(v.capture_ns,v.image_capture_ns)<=produced_ns:raise ValueError('invalid current map exposure')
        # Mutate only a bounded private copy; failures preserve prior map state.
        candidate=copy.copy(self);candidate._anchors=dict(self._anchors);candidate._retained=list(self._retained);candidate._latest=dict(self._latest)
        result=candidate._update(list(views),tuple(source_identity),produced_ns,tracking_valid,enabled)
        self.__dict__.update(candidate.__dict__);return tuple(result)
    def _update(self,views,identity,now,tracking,enabled):
        if (self._identity is not None and self._identity!=identity) or (self._tracking and not tracking):self.clear()
        if now<self._produced_ns:raise ValueError('map production clock moved backwards')
        self._produced_ns=now;self._identity=identity;self._tracking=tracking
        if not tracking:self.last_stats=dict(carved_samples=0,suppressed_gray_triangles=0);return []
        if not enabled:
            self._anchors={};self._retained=[];self._latest={};self.last_stats=dict(carved_samples=0,suppressed_gray_triangles=0);return views
        alive=lambda v: now-observation_stamp(v)<=self.retention_ms*1000000
        self._anchors={k:v for k,v in self._anchors.items() if alive(v)};self._retained=[v for v in self._retained if alive(v)]
        for v in views:
            latest=self._latest.get(v.camera_id)
            if latest is not None and (v.capture_ns<latest[0] or v.observation_id<latest[1]):raise ValueError('reordered map observation')
            if latest is not None and ((v.capture_ns==latest[0])!=(v.observation_id==latest[1])):raise ValueError('inconsistent repeated map observation')
            self._latest[v.camera_id]=(v.capture_ns,v.observation_id)
            if not np.any(v.depth):continue
            anchor=self._anchors.get(v.camera_id)
            if anchor is None:self._anchors[v.camera_id]=v
            elif _distinct(v,anchor):
                self._retained.append(replace(anchor,flags=anchor.flags|1));self._anchors[v.camera_id]=v
        present={(v.camera_id,v.observation_id) for v in views}
        unique={(v.camera_id,v.observation_id):v for v in self._retained if (v.camera_id,v.observation_id) not in present}
        ordered=sorted(unique.values(),key=lambda v:(v.image_format==3 and bool(v.flags&2),observation_stamp(v)),reverse=True)
        self._retained=ordered[:self.max_keyframes]
        carved=[];count=0
        for view in self._retained:
            view,n=_carve(view,views);count+=n
            if np.any(view.depth):carved.append(view)
        self._retained=carved
        # Anchor depth also receives contradiction evidence before it becomes history.
        for camera,anchor in list(self._anchors.items()):
            if (anchor.camera_id,anchor.observation_id) not in present:self._anchors[camera],_= _carve(anchor,views)
        retained=sorted(carved,key=lambda v:(v.camera_id,v.observation_id))
        current,suppressed=_prioritize_color(views,retained)
        self.last_stats=dict(carved_samples=count,suppressed_gray_triangles=suppressed)
        return current+retained
