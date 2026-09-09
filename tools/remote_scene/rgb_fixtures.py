"""Cross-language current/RGB/FoV/retained-map fixtures; synthetic data only."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
from .calibration import CameraCalibration,DepthFrame,Intrinsics
from .rgb_demo import six_camera_batch
from .rgb_surface import build_rgb_batch,canonical_rgb_view,prepare_rgb,prepared_metadata
from .rgb_protocol import RGBPacket,readonly
from .observation_map import RetainedObservationMap
from .protocol import pack_packet


def make_rgb_fixtures():
    c,f=six_camera_batch();fixtures={}
    def pair(c,f,**options):
        packet,_=build_rgb_batch(f,c,geometry='unprepared',source_id=0x52474244454d4f01,produced_ns=max([v.capture_ns for v in f]+[1000000000])+1000000,
                                 rig_mode=options.pop('rig_mode','posed'),synthetic=True,**options)
        prepared=replace(prepare_rgb(packet),representation=1,views=prepared_metadata(packet))
        return prepared,packet
    fixtures['six_rgb']=pair(c,f)
    current=fixtures['six_rgb'][1]
    rgb_view=current.views[-1]
    retained=tuple(replace(rgb_view,flags=rgb_view.flags|1,observation_id=i+20,capture_ns=900000000+i*10000000,image_capture_ns=900000000+i*10000000) for i in range(4))
    raw_ten=replace(current,views=current.views+retained,map_flags=3)
    fixtures['ten_views']=(replace(prepare_rgb(raw_ten),representation=1,views=prepared_metadata(raw_ten)),raw_ten)
    old_c,old_f=six_camera_batch(capture_ns=5000000000)
    fixtures['old_unusable_image']=pair(old_c,[replace(v,image_capture_ns=1,image_usable=False) if v.camera_id==5 else v for v in old_f])
    unusable_packet=fixtures['old_unusable_image'][1]
    old_depth=replace(unusable_packet.views[-1],flags=1,observation_id=2,capture_ns=4900000000)
    raw_old=replace(unusable_packet,views=unusable_packet.views+(old_depth,),map_flags=3)
    fixtures['retained_depth_only']=(replace(prepare_rgb(raw_old),representation=1,views=prepared_metadata(raw_old)),raw_old)
    fixtures['rgb_unusable']=pair(c,[replace(x,image_usable=False) if x.camera_id==5 else x for x in f])
    k=Intrinsics(6,4,10.,10.,2.5,1.5);rgb=CameraCalibration(5,k,replace(k,fx=40.,fy=40.),.001,np.eye(4),np.eye(4),image_format='RGB8')
    image=np.zeros((4,6,3),dtype='u1');image[:,:,0]=200;image[:,:,2]=np.arange(6)*40
    frame=DepthFrame(5,1,1000000000,np.full((4,6),1000,dtype='<u2'),image)
    fixtures['rgb_fov']=pair([rgb],[frame],rig_mode='stationary')
    behind=np.eye(4);behind[2,3]=-2
    clip_k=Intrinsics(3,3,10.,10.,1.,1.);clip_world=np.eye(4);clip_world[0,3]=100
    clip_c=CameraCalibration(5,clip_k,Intrinsics(3,3,.1,.1,1.,1.),.001,clip_world,np.eye(4),image_format='RGB8')
    clip_f=DepthFrame(5,1,1000000000,np.tile(np.array([1020,1020,1000],dtype='<u2'),(3,1)),np.full((3,3,3),200,dtype='u1'))
    fixtures['world_clip_depth'] = pair([clip_c],[clip_f],rig_mode='stationary')
    fixtures['rgb_behind']=pair([replace(rgb,intensity_from_depth=behind)],[frame],rig_mode='stationary')
    cache=RetainedObservationMap()
    pair(c,f,observation_map=cache,retention_enabled=True)
    moved=f[-1].world_from_rig.copy();moved[1,3]+=.2
    c2,f2=six_camera_batch(moved,capture_ns=1100000000,sequence=2)
    fixtures['retained_rgb']=pair(c2,f2,observation_map=cache,retention_enabled=True)
    empty,_=build_rgb_batch([],c2,geometry='unprepared',source_id=0x52474244454d4f01,produced_ns=1200000000,rig_mode='posed',
                            synthetic=True,observation_map=cache,retention_enabled=True)
    fixtures['retained_only']=(replace(prepare_rgb(empty),representation=1,views=prepared_metadata(empty)),empty)
    fixtures['tracking_lost']=pair(c2,f2,observation_map=cache,retention_enabled=True,tracking_valid=False)
    fixtures['invalid_depth']=pair([rgb],[replace(frame,depth=np.zeros((4,6),dtype='<u2'))],rig_mode='stationary')
    return fixtures


def write_rgb_fixtures(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);summary={}
    for name,packets in make_rgb_fixtures().items():
        summary[name]={}
        for geometry,packet in zip(('prepared','unprepared'),packets):
            raw=pack_packet(packet);(directory/f'{name}_{geometry}.rscn').write_bytes(raw)
            summary[name][geometry]=dict(packet.geometry_summary,bytes=len(raw),oldest_observation_ns=packet.oldest_observation_ns)
    (directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');return summary

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    print(json.dumps(write_rgb_fixtures(parser.parse_args().output),indent=2))
