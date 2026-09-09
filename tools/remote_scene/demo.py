"""Generate deterministic five-camera synthetic depth/Y8 and raw RSCN asset."""
import argparse
import json
import math
from pathlib import Path
import time
import zlib
import numpy as np
from .calibration import CameraCalibration,DepthFrame,Intrinsics
from .fusion import fuse_batch
from .protocol import pack_packet

OBSERVER=(-2.,0.,1.5,.5,-.5,-.5,.5)  # map_from_OpenXR_view, looking map +X


def camera_pose(position,target):
    forward=np.asarray(target,dtype=float)-position;forward/=np.linalg.norm(forward)
    right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right)
    down=np.cross(forward,right)
    matrix=np.eye(4);matrix[:3,:3]=np.column_stack((right,down,forward));matrix[:3,3]=position
    return matrix


def synthetic_batch(width=128,height=96,capture_ns=1000000000):
    focal=width/(2*math.tan(math.radians(70)/2))
    intr=Intrinsics(width,height,focal,focal,(width-1)/2,(height-1)/2)
    intr.validate()
    cameras=[];frames=[]
    positions=[(-1,0,1.5),(1,-2,1.5),(3,0,1.6),(1,2,1.6),(-.3,-1.4,2.4)]
    y,x=np.mgrid[:height,:width]
    rays=np.stack(((x-intr.ppx)/focal,(y-intr.ppy)/focal,np.ones_like(x)),axis=-1)
    for camera_id,position in enumerate(positions):
        pose=camera_pose(np.asarray(position,dtype=float),(1,0,.55))
        calibration=CameraCalibration(camera_id,intr,intr,.001,pose,np.eye(4));cameras.append(calibration)
        direction=rays@pose[:3,:3].T;origin=pose[:3,3]
        nearest=np.full((height,width),np.inf);material=np.zeros((height,width),dtype=np.uint8)
        def plane(axis,coordinate,lo,hi,value):
            t=np.divide(coordinate-origin[axis],direction[...,axis],out=np.full_like(nearest,np.inf),where=np.abs(direction[...,axis])>1e-10)
            finite=np.isfinite(t);p=origin+direction*np.where(finite,t,0)[...,None]
            valid=finite&(t>.05)&(t<nearest)&np.all(p>=np.asarray(lo)-1e-8,axis=-1)&np.all(p<=np.asarray(hi)+1e-8,axis=-1)
            nearest[valid]=t[valid];material[valid]=value
        plane(2,0,[-2,-3,0],[4,3,0],70)
        plane(0,4,[4,-3,0],[4,3,2.8],150)
        for lower,upper,value in [([.4,-.6,.55],[1.6,.6,.62],190),([.8,-.15,.62],[1.1,.15,.92],235),([1.7,-.4,0],[2.,.4,1.05],110)]:
            inv=np.divide(1.,direction,out=np.full_like(direction,np.inf),where=np.abs(direction)>1e-10)
            a=(np.asarray(lower)-origin)*inv;b=(np.asarray(upper)-origin)*inv
            near=np.max(np.minimum(a,b),axis=-1);far=np.min(np.maximum(a,b),axis=-1)
            valid=(near>.05)&(far>=near)&(near<nearest)
            nearest[valid]=near[valid];material[valid]=value
        valid=np.isfinite(nearest)&(nearest<65)
        world=origin+direction*np.where(valid,nearest,0)[...,None]
        checker=((np.floor(world[...,0]*5)+np.floor(world[...,1]*5))%2).astype(np.uint8)
        intensity=np.where(valid,np.minimum(material.astype(np.uint16)+checker*15,255),0).astype(np.uint8)
        depth=np.where(valid,np.floor(np.where(valid,nearest,0)/.001+.5),0).astype('<u2')
        frames.append(DepthFrame(camera_id,1,capture_ns+camera_id*500000,depth,intensity))
    return cameras,frames


def make_demo(width=128,height=96,*,geometry='points'):
    from .modes import build_batch
    cameras,frames=synthetic_batch(width,height)
    return build_batch(frames,cameras,geometry=geometry,source_id=0x5253434e44454d4f,world_epoch=1,sequence=1,calibration_id=1,
                      produced_ns=1005000000,rig_mode='stationary',observer_pose=OBSERVER,synthetic=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--width',type=int,default=128);parser.add_argument('--height',type=int,default=96)
    parser.add_argument('--recording-dir',type=Path,help='Also write the exact five synthetic views and strict replay manifest')
    parser.add_argument('--geometry',choices=('points','prepared','unprepared'),default='points')
    parser.add_argument('--all-modes',action='store_true',help='Also write legacy surfaces and RGB retained-gimbal prepared/unprepared assets')
    args=parser.parse_args();started=time.perf_counter()
    if args.all_modes and args.geometry!='points':raise ValueError('--all-modes requires the legacy points primary output')
    packet,stats=make_demo(args.width,args.height,geometry=args.geometry);raw=pack_packet(packet)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    temporary=args.output.with_name(args.output.name+'.pending');temporary.write_bytes(raw);temporary.replace(args.output)
    if args.all_modes:
        for mode in ('prepared','unprepared'):
            candidate,_=make_demo(args.width,args.height,geometry=mode)
            output=args.output.with_name(f'demo-{mode}.rscn')
            pending=output.with_name(output.name+'.pending');pending.write_bytes(pack_packet(candidate));pending.replace(output)
        from .gimbal_demo import make_gimbal_demo
        for mode in ('prepared','unprepared'):
            candidate,_=make_gimbal_demo(geometry=mode)
            output=args.output.with_name(f'demo-rgb-{mode}.rscn')
            pending=output.with_name(output.name+'.pending');pending.write_bytes(pack_packet(candidate));pending.replace(output)
    if args.recording_dir:
        from .recording import write_recording
        cameras,frames=synthetic_batch(args.width,args.height)
        write_recording(args.recording_dir,cameras,[(1005000000,frames)],source_id=packet.source_id,
                        world_epoch=1,calibration_id=1,rig_mode='stationary',observer_pose=OBSERVER,synthetic=True)
    print(json.dumps(dict(stats,raw_bytes=len(raw),zlib_bytes=len(zlib.compress(raw,1)),host_seconds=time.perf_counter()-started,
                          source='synthetic five-view depth/Y8; no camera or Quest performance claim')))


if __name__=='__main__':main()
