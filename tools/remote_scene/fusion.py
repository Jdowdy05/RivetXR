"""Deterministic current-batch voxel means; no temporal map or pose inference."""
import numpy as np
from itertools import islice
from .calibration import project_frame
from .protocol import Packet,POINT_DTYPE,MAX_POINTS,PARTIAL,TRUNCATED,RECORDED,SYNTHETIC,integer,validate


def fuse_batch(frames,calibrations,*,source_id=1,world_epoch=1,sequence=1,calibration_id=1,
               produced_ns,rig_mode,voxel_size_m=.01,max_capture_skew_ms=20,max_points=MAX_POINTS,
               observer_pose=(0,0,1.5,0,0,0,1),recorded=False,synthetic=False):
    integer(max_points,1,MAX_POINTS,'point budget');integer(max_capture_skew_ms,1,100,'capture skew')
    integer(produced_ns,1,2**64-1,'production time')
    if rig_mode not in ('stationary','posed'):raise ValueError('explicit rig mode required')
    voxel=float(np.float32(voxel_size_m))
    if not np.isfinite(voxel) or not float(np.float32(.001))<=voxel<=float(np.float32(.1)):raise ValueError('invalid voxel size')
    cameras={c.camera_id:c for c in calibrations}
    if not cameras or len(cameras)!=len(calibrations) or len(cameras)>5 or any(c.camera_id>4 or c.image_format!='Y8' for c in calibrations):raise ValueError('invalid calibrated camera set')
    expected=sum(1<<i for i in cameras)
    frames=list(islice(frames,6))
    if len(frames)>5 or len({f.camera_id for f in frames})!=len(frames):raise ValueError('duplicate/too many frames')
    for frame in frames:
        if frame.camera_id not in cameras:raise ValueError('unknown camera')
        frame.validate(cameras[frame.camera_id],rig_mode)
        if frame.capture_ns>produced_ns:raise ValueError('capture is newer than production time')
    newest=max((f.capture_ns for f in frames),default=0)
    accepted=sorted((f for f in frames if newest-f.capture_ns<=max_capture_skew_ms*1000000
                     and produced_ns-f.capture_ns<=2000000000),key=lambda f:f.camera_id)
    xyz,gray,ids=[],[],[]
    for frame in accepted:
        p,g=project_frame(frame,cameras[frame.camera_id],rig_mode)
        xyz.append(p);gray.append(g);ids.append(np.full(len(p),frame.camera_id,dtype=np.uint8))
    mask=sum(1<<f.camera_id for f in accepted)
    flags=(RECORDED if recorded else 0)|(SYNTHETIC if synthetic else 0)|(PARTIAL if mask!=expected else 0)
    source_points=sum(map(len,xyz));voxel_count=0
    points=np.empty(0,dtype=POINT_DTYPE)
    if source_points:
        p=np.concatenate(xyz);g=np.concatenate(gray);camera=np.concatenate(ids)
        key=np.floor(p/voxel).astype(np.int64)
        unique,inverse,counts=np.unique(key,axis=0,return_inverse=True,return_counts=True)
        voxel_count=len(unique)
        sums=np.zeros((voxel_count,3),dtype=np.float64);np.add.at(sums,inverse,p)
        intensities=np.bincount(inverse,weights=g,minlength=voxel_count)/counts
        # Count distinct cameras, not the number of contributing pixels.
        pairs=np.unique(np.column_stack((inverse,camera)),axis=0)
        support=np.bincount(pairs[:,0],minlength=voxel_count)
        if voxel_count>max_points:
            selection=(np.arange(max_points,dtype=np.int64)*voxel_count)//max_points;flags|=TRUNCATED
        else:selection=np.arange(voxel_count)
        points=np.zeros(len(selection),dtype=POINT_DTYPE)
        points['xyz']=(sums/counts[:,None])[selection].astype('<f4')
        points['gray']=np.floor(intensities[selection]+.5).astype(np.uint8)
        points['support']=support[selection].astype(np.uint8)
        points['radius_mm']=max(1,min(50,int(np.floor(voxel*500+.5))))
    packet=Packet(source_id,world_epoch,sequence,calibration_id,
        min((f.capture_ns for f in accepted),default=0),max((f.capture_ns for f in accepted),default=0),
        produced_ns,expected,mask,flags,voxel,max_capture_skew_ms,tuple(observer_pose),points)
    validate(packet)
    return packet,dict(input_points=source_points,voxels=voxel_count,points=len(points),
                       expected_cameras=len(cameras),contributing_cameras=len(accepted),
                       omitted_camera_ids=sorted(set(cameras)-{f.camera_id for f in accepted}),rig_mode=rig_mode)
