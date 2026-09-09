"""Deterministic cross-language RSCN surface fixtures; writes only on explicit CLI use."""
import argparse
import json
from pathlib import Path
import numpy as np
from .calibration import CameraCalibration,DepthFrame,Intrinsics
from .demo import synthetic_batch,OBSERVER
from .surface import build_surface_batch
from .protocol import pack_packet


def fixture_batches():
    """Return small analytical and five-view batches with nontrivial calibration."""
    intr=Intrinsics(8,6,18.,17.,3.5,2.5)
    gray=((np.arange(48).reshape(6,8)*5)%256).astype('u1')
    c=CameraCalibration(0,intr,intr,.001,np.eye(4),np.eye(4))
    def frame(depth,pose=None,camera_id=0,capture=1000000000):
        return DepthFrame(camera_id,1,capture,depth,gray,pose)
    plane=np.full((6,8),1000,dtype='<u2')
    step=plane.copy();step[:,4:]=1500;step[2,1]=0
    slope=plane+np.arange(8,dtype='<u2')[None,:]*10
    intensity_transform=np.eye(4);angle=.03
    intensity_transform[:3,:3]=[[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]]
    intensity_transform[:3,3]=[.01,.003,.04]
    projective=CameraCalibration(0,intr,Intrinsics(8,6,15.5,15.,3.1,2.3),.001,np.eye(4),intensity_transform)
    world=np.eye(4);angle=.2;world[:3,:3]=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
    world[:3,3]=[.123456789,-.25,.4]
    second=CameraCalibration(2,intr,intr,.001,np.eye(4),np.eye(4))
    world2=world.copy();world2[:3,3]+=[.4,.2,0]
    clip_intr=Intrinsics(3,3,10.,10.,1.,1.)
    clip_world=np.eye(4);clip_world[0,3]=100
    clip_camera=CameraCalibration(0,clip_intr,Intrinsics(3,3,.1,.1,1.,1.),.001,clip_world,np.eye(4))
    clip_depth=np.tile(np.array([1020,1020,1000],dtype='<u2'),(3,1))
    clip_frame=DepthFrame(0,1,1000000000,clip_depth,np.arange(9,dtype='u1').reshape(3,3))
    cameras,frames=synthetic_batch(128,96)
    return {
        'plane':([c],[frame(plane)],'stationary'),
        'projective':([projective],[frame(slope)],'stationary'),
        'depth_step':([c],[frame(step)],'stationary'),
        'moving':([c,second],[frame(plane,world),frame(slope,world2,2,1001000000)],'posed'),
        'invalid_depth':([c],[frame(np.zeros_like(plane))],'stationary'),
        'no_cameras':([c],[],'stationary'),
        'five_views':(cameras,frames,'stationary'),
        'world_clip':([clip_camera],[clip_frame],'stationary'),
    }


def make_fixtures():
    """Map fixture names to (prepared_packet, unprepared_packet)."""
    return {name:tuple(build_surface_batch(frames,cameras,geometry=mode,source_id=0x5355524641434501,
               produced_ns=1005000000,rig_mode=rig,observer_pose=OBSERVER,synthetic=True)[0]
               for mode in ('prepared','unprepared'))
            for name,(cameras,frames,rig) in fixture_batches().items()}


def write_fixtures(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);summary={}
    for name,packets in make_fixtures().items():
        summary[name]={}
        for mode,packet in zip(('prepared','unprepared'),packets):
            raw=pack_packet(packet);(directory/f'{name}_{mode}.rscn').write_bytes(raw)
            summary[name][mode]=dict(packet.geometry_summary,bytes=len(raw),mask=packet.contributing_mask,flags=packet.flags)
    (directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True,type=Path)
    print(json.dumps(write_fixtures(parser.parse_args().output),indent=2))


if __name__=='__main__':main()
