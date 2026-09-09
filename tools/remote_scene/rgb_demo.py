"""Six synthetic camera observations, including actual raycast RGB from a gimbal pose."""
from dataclasses import replace
import numpy as np
from .calibration import CameraCalibration,DepthFrame,Intrinsics,rigid
from .demo import synthetic_batch,camera_pose


def _render(intr,world_from_camera,rgb=False):
    y,x=np.mgrid[:intr.height,:intr.width]
    rays=np.stack(((x-intr.ppx)/intr.fx,(y-intr.ppy)/intr.fy,np.ones_like(x)),axis=-1)
    direction=rays@world_from_camera[:3,:3].T;origin=world_from_camera[:3,3]
    nearest=np.full((intr.height,intr.width),np.inf);material=np.zeros((intr.height,intr.width),dtype='u1')
    def plane(axis,value,lower,upper,mat):
        t=np.divide(value-origin[axis],direction[...,axis],out=np.full_like(nearest,np.inf),where=np.abs(direction[...,axis])>1e-10)
        finite=np.isfinite(t);p=origin+direction*np.where(finite,t,0)[...,None]
        valid=finite&(t>.05)&(t<nearest)&np.all(p>=np.asarray(lower)-1e-8,axis=-1)&np.all(p<=np.asarray(upper)+1e-8,axis=-1)
        nearest[valid]=t[valid];material[valid]=mat
    plane(2,0,[-2,-3,0],[4,3,0],1);plane(0,4,[4,-3,0],[4,3,2.8],2)
    for lower,upper,mat in [([.4,-.6,.55],[1.6,.6,.62],3),([.8,-.15,.62],[1.1,.15,.92],4),([1.7,-.4,0],[2.,.4,1.05],5)]:
        inv=np.divide(1.,direction,out=np.full_like(direction,np.inf),where=np.abs(direction)>1e-10)
        a=(np.asarray(lower)-origin)*inv;b=(np.asarray(upper)-origin)*inv
        near=np.max(np.minimum(a,b),axis=-1);far=np.min(np.maximum(a,b),axis=-1)
        valid=(near>.05)&(far>=near)&(near<nearest);nearest[valid]=near[valid];material[valid]=mat
    valid=np.isfinite(nearest)&(nearest<65)
    world=origin+direction*np.where(valid,nearest,0)[...,None]
    checker=((np.floor(world[...,0]*5)+np.floor(world[...,1]*5))%2).astype('u1')
    palette=np.array([[0,0,0],[70,85,100],[145,160,175],[180,115,65],[235,45,35],[40,155,215]],dtype='u1')
    image=np.minimum(palette[material].astype('u2')+checker[:,:,None]*15,255).astype('u1');image[~valid]=0
    depth=np.floor(np.where(valid,nearest,0)*1000+.5).astype('<u2')
    return depth,image if rgb else image[:,:,0]


def six_camera_batch(gimbal_world_from_depth=None,capture_ns=1000000000,sequence=1):
    if gimbal_world_from_depth is None:gimbal_world_from_depth=camera_pose(np.array([-1.2,0,1.5]),(1.,0.,.65))
    pose=rigid(gimbal_world_from_depth)
    cameras,frames=synthetic_batch(128,96,capture_ns)
    frames=[replace(f,sequence=sequence,capture_ns=capture_ns,world_from_rig=np.eye(4)) for f in frames]
    width,height=128,96;focal=width/(2*np.tan(np.deg2rad(43.5)))
    depth_k=Intrinsics(width,height,float(focal),float(focal),(width-1)/2,(height-1)/2)
    iw,ih=256,144;image_f=iw/(2*np.tan(np.deg2rad(34.5)))
    image_k=Intrinsics(iw,ih,float(image_f),float(image_f),(iw-1)/2,(ih-1)/2)
    image_from_depth=np.eye(4);image_from_depth[0,3]=-.015
    depth,_=_render(depth_k,pose);_,image=_render(image_k,pose@np.linalg.inv(image_from_depth),rgb=True)
    cameras.append(CameraCalibration(5,depth_k,image_k,.001,np.eye(4),image_from_depth,image_format='RGB8'))
    frames.append(DepthFrame(5,sequence,capture_ns,depth,image,pose,image_capture_ns=capture_ns))
    return cameras,frames
