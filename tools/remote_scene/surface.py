"""Current-batch depth-grid surface patches with calibrated projective Y8 texture.

No cross-camera triangles, temporal map, completed hidden surfaces, or colliders.
The reduced float32 wire inputs are canonical for both reconstruction locations.
"""
from dataclasses import replace
from itertools import islice
import numpy as np
from .calibration import require_legacy
from .protocol import PARTIAL,TRUNCATED,RECORDED,SYNTHETIC,integer
from .surface_protocol import (SurfaceView,SurfacePacket,PREPARED,UNPREPARED,readonly,
                               validate_surface,validate_view,validate_mesh)


def _reduced(image,intrinsics,max_width,max_height):
    height,width=image.shape[:2]
    if min(height,width)<2:raise ValueError('surface images require at least two pixels per dimension')
    scale=min(1.,max_width/width,max_height/height)
    outw=max(2,min(max_width,int(np.floor(width*scale+.5))))
    outh=max(2,min(max_height,int(np.floor(height*scale+.5))))
    # floor(center+.5) is nearest-neighbor with ties rounded upward.
    x=np.minimum(width-1,np.floor((np.arange(outw)+.5)*width/outw).astype(np.int64))
    y=np.minimum(height-1,np.floor((np.arange(outh)+.5)*height/outh).astype(np.int64))
    rx,ry=outw/width,outh/height
    intr=tuple(float(x) for x in np.asarray((intrinsics.fx*rx,intrinsics.fy*ry,
                  (intrinsics.ppx+.5)*rx-.5,(intrinsics.ppy+.5)*ry-.5),dtype=np.float32))
    return readonly(image[np.ix_(y,x)]),intr,(outw!=width or outh!=height)


def _canonical_view(frame,calibration,rig_mode):
    depth,di,dr=_reduced(frame.depth,calibration.depth,64,48)
    intensity,ii,ir=_reduced(frame.intensity,calibration.intensity,320,240)
    world=calibration.rig_from_depth
    if rig_mode=='posed':world=np.asarray(frame.world_from_rig,dtype=np.float64).reshape(4,4)@world
    view=SurfaceView(frame.camera_id,di,ii,float(np.float32(calibration.depth_units_m)),readonly(world[:3],'<f4'),
        readonly(calibration.intensity_from_depth[:3],'<f4'),frame.capture_ns,depth,intensity)
    validate_view(view,frame.capture_ns,frame.capture_ns)
    return view,dr or ir


def _mesh_for_view(view,tile_offset,atlas_width,atlas_height):
    h,w=view.depth.shape;y,x=np.mgrid[:h,:w];fx,fy,px,py=np.asarray(view.depth_intrinsics,dtype=np.float32).astype(np.float64)
    z=view.depth.reshape(-1).astype(np.float64)*float(np.float32(view.depth_units_m))
    points=np.column_stack(((x.reshape(-1)-px)*z/fx,(y.reshape(-1)-py)*z/fy,z))
    transform=np.asarray(view.intensity_from_depth,dtype=np.float32).astype(np.float64)
    ir=points@transform[:,:3].T+transform[:,3]
    fx,fy,px,py=np.asarray(view.intensity_intrinsics,dtype=np.float32).astype(np.float64);ih,iw=view.intensity.shape
    valid=(view.depth.reshape(-1)!=0)&np.isfinite(ir).all(axis=1)&(ir[:,2]>=1e-6)&(ir[:,2]<=1024)
    selected=np.flatnonzero(valid)
    uv=ir[selected,:2]/ir[selected,2,None]*np.array([fx,fy])+[px,py]
    keep=np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,0]<=iw-1)&(uv[:,1]>=0)&(uv[:,1]<=ih-1)
    selected=selected[keep];uv=uv[keep]
    pixels=np.floor(uv+.5).astype(np.int64);flat=pixels[:,1]*iw+pixels[:,0]
    nearest=np.full(iw*ih,np.inf,dtype=np.float64)
    np.minimum.at(nearest,flat,ir[selected,2])
    selected=selected[ir[selected,2]<=nearest[flat]+.005]
    world_transform=np.asarray(view.world_from_depth,dtype=np.float32).astype(np.float64)
    world=points@world_transform[:,:3].T+world_transform[:,3]
    valid=np.zeros(len(points),dtype=bool);valid[selected]=True
    valid &= np.isfinite(world).all(axis=1)&np.all(np.abs(world)<=100,axis=1)
    grid=np.arange(w*h).reshape(h,w)
    a=grid[:-1,:-1].reshape(-1);b=grid[:-1,1:].reshape(-1)
    c=grid[1:,:-1].reshape(-1);d=grid[1:,1:].reshape(-1)
    triangles=np.stack((np.column_stack((a,c,b)),np.column_stack((b,c,d))),axis=1).reshape(-1,3)
    triangles=triangles[np.all(valid[triangles],axis=1)]
    if len(triangles):
        corners=world[triangles];depths=z[triangles]
        keep=np.ones(len(triangles),dtype=bool)
        for i,j in ((0,1),(1,2),(2,0)):
            edge=corners[:,i]-corners[:,j]
            keep &= np.sum(edge*edge,axis=1)<=.5**2
            keep &= np.abs(depths[:,i]-depths[:,j])<=.05+.02*np.minimum(depths[:,i],depths[:,j])
        area=np.cross(corners[:,1]-corners[:,0],corners[:,2]-corners[:,0])
        keep &= np.sum(area*area,axis=1)>1e-12
        # Final float32 positions must remain nondegenerate on the receiver.
        emitted=corners.astype(np.float32).astype(np.float64)
        area=np.cross(emitted[:,1]-emitted[:,0],emitted[:,2]-emitted[:,0])
        keep &= np.sum(area*area,axis=1)>1e-12
        triangles=triangles[keep]
    used=np.unique(triangles)
    vertices=np.empty((len(used),6),dtype='<f4')
    vertices[:,:3]=world[used]
    q=ir[used,2]
    vertices[:,3]=(fx*ir[used,0]+(px+.5+tile_offset)*q)/atlas_width
    vertices[:,4]=(fy*ir[used,1]+(py+.5)*q)/atlas_height
    vertices[:,5]=q
    mapping=np.full(len(points),-1,dtype=np.int64);mapping[used]=np.arange(len(used))
    return vertices,mapping[triangles].reshape(-1)


def prepare_surface(packet):
    """Convert bounded canonical views; preserve raw representation identity.

    The returned packet retains views so re-encoding representation 2 still
    transmits depth/Y8, while geometry_summary describes its prepared buffers.
    """
    validate_surface(packet)
    if packet.representation==PREPARED:return packet
    width=sum(v.intensity.shape[1] for v in packet.views)
    height=max((v.intensity.shape[0] for v in packet.views),default=0)
    atlas=np.zeros((height,width),dtype='u1');vertices=[];indices=[];offset=0;vertex_offset=0
    for view in packet.views:
        ih,iw=view.intensity.shape;atlas[:ih,offset:offset+iw]=view.intensity
        v,i=_mesh_for_view(view,offset,width,height)
        vertices.append(v);indices.append(i+vertex_offset);vertex_offset+=len(v);offset+=iw
    if vertex_offset:
        vertices=np.concatenate(vertices);indices=np.concatenate(indices).astype('<u2')
    else:
        vertices=np.empty((0,6),dtype='<f4');indices=np.empty(0,dtype='<u2');atlas=np.empty((0,0),dtype='u1')
    validate_mesh(vertices,indices,atlas)
    return replace(packet,vertices=readonly(vertices),indices=readonly(indices),atlas=readonly(atlas))


def build_surface_batch(frames,calibrations,*,geometry='prepared',source_id=1,world_epoch=1,sequence=1,calibration_id=1,
                        produced_ns,rig_mode,max_capture_skew_ms=20,observer_pose=(0,0,1.5,0,0,0,1),recorded=False,synthetic=False):
    """Build one bounded scene batch, selecting robot or receiver reconstruction."""
    if geometry not in ('prepared','unprepared'):raise ValueError('geometry must be prepared or unprepared')
    integer(max_capture_skew_ms,1,100,'capture skew');integer(produced_ns,1,2**64-1,'production time')
    if rig_mode not in ('stationary','posed'):raise ValueError('explicit rig mode required')
    calibrations=list(islice(calibrations,6));cameras={c.camera_id:c for c in calibrations}
    if not cameras or len(cameras)!=len(calibrations) or len(cameras)>5 or any(c.camera_id>4 or c.image_format!='Y8' for c in calibrations):raise ValueError('invalid calibrated camera set')
    frames=list(islice(frames,6))
    if len(frames)>5 or len({f.camera_id for f in frames})!=len(frames):raise ValueError('duplicate/too many frames')
    for frame in frames:
        if frame.camera_id not in cameras:raise ValueError('unknown camera')
        require_legacy(frame,cameras[frame.camera_id])
        frame.validate(cameras[frame.camera_id],rig_mode)
        if frame.capture_ns>produced_ns:raise ValueError('capture is newer than production time')
    newest=max((f.capture_ns for f in frames),default=0)
    accepted=sorted((f for f in frames if newest-f.capture_ns<=max_capture_skew_ms*1000000
                    and produced_ns-f.capture_ns<=2000000000),key=lambda f:f.camera_id)
    expected=sum(1<<i for i in cameras);mask=sum(1<<f.camera_id for f in accepted)
    flags=(RECORDED if recorded else 0)|(SYNTHETIC if synthetic else 0)|(PARTIAL if mask!=expected else 0)
    views=[]
    for frame in accepted:
        view,truncated=_canonical_view(frame,cameras[frame.camera_id],rig_mode);views.append(view)
        if truncated:flags|=TRUNCATED
    packet=SurfacePacket(source_id,world_epoch,sequence,calibration_id,min((f.capture_ns for f in accepted),default=0),
        max((f.capture_ns for f in accepted),default=0),produced_ns,expected,mask,flags,max_capture_skew_ms,tuple(observer_pose),UNPREPARED,
        readonly(np.empty((0,6)),'<f4'),readonly([],'<u2'),readonly(np.empty((0,0)),'u1'),tuple(views))
    validate_surface(packet)
    if geometry=='prepared':
        packet=replace(prepare_surface(packet),representation=PREPARED,views=())
        validate_surface(packet)
    stats=dict(packet.geometry_summary,geometry=geometry,expected_cameras=len(cameras),contributing_cameras=len(accepted),
               omitted_camera_ids=sorted(set(cameras)-{f.camera_id for f in accepted}),rig_mode=rig_mode,
               depth_samples=sum(v.depth.size for v in views),intensity_pixels=sum(v.intensity.size for v in views))
    return packet,stats
