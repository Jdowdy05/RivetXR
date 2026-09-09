"""Canonical RGB-D observation geometry, retaining depth outside image coverage."""
from dataclasses import replace
from itertools import islice
import numpy as np
from .calibration import rigid
from .protocol import integer,PARTIAL,TRUNCATED,RECORDED,SYNTHETIC
from .surface import _reduced
from .rgb_protocol import RGBView,RGBMeta,RGBPacket,readonly,full_mask,shelf,validate_rgb


def canonical_rgb_view(frame,calibration,rig_mode):
    frame.validate(calibration,rig_mode)
    depth,dk,dr=_reduced(frame.depth,calibration.depth,64,48)
    image,ik,ir=_reduced(frame.intensity,calibration.intensity,256,192)
    world=calibration.rig_from_depth
    if rig_mode=='posed':world=rigid(frame.world_from_rig)@world
    return RGBView(frame.camera_id,dk,ik,float(np.float32(calibration.depth_units_m)),readonly(world[:3],'<f4'),
        readonly(frame.image_pose(calibration)[:3],'<f4'),frame.capture_ns,depth,image,frame.image_timestamp_ns,frame.sequence,
        2 if frame.image_usable else 0,3 if calibration.image_format=='RGB8' else 1,full_mask(depth.shape[1],depth.shape[0])),dr or ir


def grid_geometry(view):
    """World positions, image coordinates/visibility, candidate triangles and keep mask."""
    h,w=view.depth.shape;y,x=np.mgrid[:h,:w];fx,fy,px,py=np.asarray(view.depth_intrinsics,dtype='f4').astype(float)
    depth=view.depth.reshape(-1).astype(float)*float(np.float32(view.depth_units_m))
    local=np.column_stack(((x.reshape(-1)-px)*depth/fx,(y.reshape(-1)-py)*depth/fy,depth))
    transform=np.asarray(view.world_from_depth,dtype='f4').astype(float);world=local@transform[:,:3].T+transform[:,3]
    transform=np.asarray(view.intensity_from_depth,dtype='f4').astype(float);image=local@transform[:,:3].T+transform[:,3]
    ik=np.asarray(view.intensity_intrinsics,dtype='f4').astype(float)
    texvalid=(depth>0)&np.isfinite(image).all(axis=1)&bool(view.flags&2);ih,iw=view.intensity.shape[:2]
    good=(depth>0)&np.isfinite(image).all(axis=1)&(image[:,2]>0)
    selected=np.flatnonzero(good)
    uv=image[selected,:2]/image[selected,2,None]*ik[:2]+ik[2:]
    inside=np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,0]<=iw-1)&(uv[:,1]>=0)&(uv[:,1]<=ih-1)
    selected=selected[inside];uv=uv[inside];pixels=np.floor(uv+.5).astype(int)
    flat=pixels[:,1]*iw+pixels[:,0];nearest=np.full(iw*ih,np.inf)
    np.minimum.at(nearest,flat,image[selected,2])
    texvalid[selected[image[selected,2]>nearest[flat]+.005]]=False
    valid=(depth>0)&np.isfinite(world).all(axis=1)&np.all(np.abs(world)<=100,axis=1)
    grid=np.arange(w*h).reshape(h,w);a=grid[:-1,:-1].reshape(-1);b=a+1;c=a+w;d=c+1
    triangles=np.stack((np.column_stack((a,c,b)),np.column_stack((b,c,d))),axis=1).reshape(-1,3)
    keep=np.unpackbits(np.frombuffer(view.triangle_keep,dtype='u1'),bitorder='little')[:len(triangles)].astype(bool)
    keep &= np.all(valid[triangles],axis=1)
    p=world[triangles];z=depth[triangles]
    for u,v in ((0,1),(1,2),(2,0)):
        edge=p[:,u]-p[:,v];keep &= np.sum(edge*edge,axis=1)<=.25
        keep &= np.abs(z[:,u]-z[:,v])<=.05+.02*np.minimum(z[:,u],z[:,v])
    for emitted in (p,p.astype('f4').astype(float)):
        cross=np.cross(emitted[:,1]-emitted[:,0],emitted[:,2]-emitted[:,0]);keep &= np.sum(cross*cross,axis=1)>1e-12
    return world,image,texvalid,triangles,keep


def prepare_rgb(packet):
    validate_rgb(packet)
    if packet.representation==1:return packet
    dimensions=[(v.intensity.shape[1],v.intensity.shape[0]) for v in packet.views]
    width,height,tiles=shelf(dimensions);atlas=np.zeros((height,width,3),dtype='u1')
    all_vertices=[];all_indices=[];metas=[];vertex_count=index_count=0
    for view,tile in zip(packet.views,tiles):
        tx,ty,tw,th=tile
        image=view.intensity if view.image_format==3 else np.repeat(view.intensity[:,:,None],3,axis=2)
        atlas[ty:ty+th,tx:tx+tw]=image
        world,ip,valid,triangles,keep=grid_geometry(view);triangles=triangles[keep];used=np.unique(triangles)
        vertices=np.empty((len(used),11),dtype='<f4');vertices[:,:3]=world[used]
        fx,fy,px,py=np.asarray(view.intensity_intrinsics,dtype='f4').astype(float);q=ip[used,2]
        uvq=np.column_stack(((fx*ip[used,0]+(px+.5+tx)*q)/width,(fy*ip[used,1]+(py+.5+ty)*q)/height,q))
        finite=np.isfinite(uvq).all(axis=1)&np.all(np.abs(uvq)<=np.finfo(np.float32).max,axis=1)
        uvq[~finite]=0 # Undefined/nonrepresentable projection, with explicit untextured validity.
        vertices[:,3:6]=uvq
        vertices[:,6:10]=np.asarray(((tx+.5)/width,(ty+.5)/height,(tx+tw-.5)/width,(ty+th-.5)/height),dtype='<f4')
        vertices[:,10]=(valid[used]&finite).astype('f4')
        mapping=np.full(len(world),-1,dtype=int);mapping[used]=np.arange(len(used))+vertex_count
        indices=mapping[triangles].reshape(-1).astype('<u2')
        metas.append(RGBMeta(view.camera_id,view.flags,view.observation_id,view.capture_ns,view.image_capture_ns,
                            vertex_count,len(vertices),index_count,len(indices),*tile,view.image_format))
        all_vertices.append(vertices);all_indices.append(indices);vertex_count+=len(vertices);index_count+=len(indices)
    vertices=np.concatenate(all_vertices) if all_vertices else np.empty((0,11),dtype='<f4')
    indices=np.concatenate(all_indices) if all_indices else np.empty(0,dtype='<u2')
    result=replace(packet,vertices=readonly(vertices),indices=readonly(indices),atlas=readonly(atlas),prepared_views=tuple(metas))
    # Preserve raw views/identity for callers; prepared serialization uses metadata.
    prepared=replace(result,representation=1,views=tuple(metas));validate_rgb(prepared)
    return result


def prepared_metadata(raw_packet):
    if raw_packet.prepared_views:return raw_packet.prepared_views
    dimensions=[(v.intensity.shape[1],v.intensity.shape[0]) for v in raw_packet.views]
    _,_,tiles=shelf(dimensions);metas=[];nv=ni=0
    for view,tile in zip(raw_packet.views,tiles):
        world,image,valid,triangles,keep=grid_geometry(view);triangles=triangles[keep];vertices=len(np.unique(triangles));indices=triangles.size
        metas.append(RGBMeta(view.camera_id,view.flags,view.observation_id,view.capture_ns,view.image_capture_ns,nv,vertices,ni,indices,*tile,view.image_format))
        nv+=vertices;ni+=indices
    return tuple(metas)


def build_rgb_batch(frames,calibrations,*,geometry='prepared',source_id=1,world_epoch=1,sequence=1,calibration_id=1,
                    produced_ns,rig_mode,max_capture_skew_ms=20,observer_pose=(0,0,1.5,0,0,0,1),recorded=False,synthetic=False,
                    observation_map=None,tracking_valid=True,retention_enabled=False,retention_ms=30000):
    from .observation_map import RetainedObservationMap
    if geometry not in ('prepared','unprepared'):raise ValueError('RGB geometry must be prepared or unprepared')
    integer(produced_ns,1,2**64-1,'production time');integer(max_capture_skew_ms,1,100,'capture skew')
    integer(retention_ms,1,60000,'retention milliseconds')
    if rig_mode not in ('stationary','posed') or type(tracking_valid) is not bool or type(retention_enabled) is not bool:raise ValueError('invalid tracking/rig declaration')
    calibrations=list(islice(calibrations,7));cameras={c.camera_id:c for c in calibrations}
    if not cameras or len(cameras)!=len(calibrations) or len(cameras)>6:raise ValueError('invalid RGB camera set')
    frames=list(islice(frames,7))
    if len(frames)>6 or len({f.camera_id for f in frames})!=len(frames):raise ValueError('duplicate/too many RGB frames')
    for frame in frames:
        if frame.camera_id not in cameras:raise ValueError('unknown RGB camera')
        frame.validate(cameras[frame.camera_id],rig_mode)
        if max(frame.capture_ns,frame.image_timestamp_ns)>produced_ns:raise ValueError('exposure newer than production')
    newest=max((f.capture_ns for f in frames),default=0)
    accepted=sorted((f for f in frames if newest-f.capture_ns<=max_capture_skew_ms*1000000 and produced_ns-f.capture_ns<=2000000000
                     and (not f.image_usable or produced_ns-f.image_timestamp_ns<=2000000000)),key=lambda f:f.camera_id) if tracking_valid else []
    flags=(RECORDED if recorded else 0)|(SYNTHETIC if synthetic else 0);views=[]
    for frame in accepted:
        view,truncated=canonical_rgb_view(frame,cameras[frame.camera_id],rig_mode);views.append(view)
        if truncated:flags|=TRUNCATED
    cache=observation_map if observation_map is not None else RetainedObservationMap(retention_ms=retention_ms)
    if cache.retention_ms!=retention_ms:raise ValueError('map and packet retention policies differ')
    expected=sum(1<<i for i in cameras)
    def make_packet(observations,generation):
        current=[v for v in observations if not v.flags&1]
        mask=sum(1<<v.camera_id for v in current)
        return RGBPacket(source_id,world_epoch,sequence,calibration_id,min((v.capture_ns for v in current),default=0),
            max((v.capture_ns for v in current),default=0),produced_ns,expected,mask,flags|(PARTIAL if mask!=expected else 0),
            max_capture_skew_ms,tuple(observer_pose),2,readonly(np.empty((0,11)),'<f4'),readonly([],'<u2'),
            readonly(np.empty((0,0,3)),'u1'),tuple(observations),generation,retention_ms,int(tracking_valid)|(int(retention_enabled)<<1))
    # Reject malformed public metadata before a persistent map changes identity.
    validate_rgb(make_packet(views,cache.map_generation))
    views=cache.update(views,(source_id,world_epoch,calibration_id),produced_ns,tracking_valid=tracking_valid,enabled=retention_enabled)
    packet=make_packet(views,cache.map_generation)
    current=[v for v in views if not v.flags&1]
    validate_rgb(packet)
    if geometry=='prepared':
        packet=prepare_rgb(packet);packet=replace(packet,representation=1,views=packet.prepared_views);validate_rgb(packet)
    return packet,dict(packet.geometry_summary,geometry=geometry,contributing_cameras=len(current),expected_cameras=len(cameras),
                       omitted_camera_ids=sorted(set(cameras)-{v.camera_id for v in current}),rig_mode=rig_mode,
                       retention_enabled=retention_enabled,tracking_valid=tracking_valid,**cache.last_stats)
