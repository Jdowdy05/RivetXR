"""Synthetic inspection showcase through the real remote RGB-D scene protocol.

Depth comes only from analytic geometry. A supplied image is used as planar
workbench albedo. Output is a host software preview, never a Quest capture.
Run from the repository root; requires NumPy and Pillow, not Newton or cameras.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
import sys
import PIL
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from tools.remote_scene.calibration import CameraCalibration,DepthFrame,Intrinsics
from tools.remote_scene.demo import camera_pose,OBSERVER
from tools.remote_scene.rgb_surface import build_rgb_batch,prepare_rgb
from tools.remote_scene.observation_map import RetainedObservationMap
from tools.remote_scene.protocol import pack_packet,unpack_packet


def box_hit(origin,directions,lower,upper):
    origin=np.asarray(origin,dtype=float);d=np.asarray(directions,dtype=float)
    safe=np.abs(d)>1e-12
    t0=np.divide(lower-origin,d,out=np.full_like(d,-np.inf),where=safe)
    t1=np.divide(upper-origin,d,out=np.full_like(d,np.inf),where=safe)
    low=np.minimum(t0,t1);high=np.maximum(t0,t1)
    outside=(~safe)&((origin<lower)|(origin>upper))
    near=np.max(low,axis=-1);far=np.min(high,axis=-1)
    valid=(far>=np.maximum(near,1e-5))&~np.any(outside,axis=-1)
    t=np.where(valid,np.where(near>1e-5,near,far),np.inf)
    point=origin+d*np.where(valid,t,0)[...,None]
    face=np.argmin(np.concatenate((np.abs(point-lower),np.abs(point-upper)),axis=-1),axis=-1)
    normal=np.zeros_like(d);rows=np.arange(len(d));normal[rows,face%3]=np.where(face<3,-1.,1.)
    normal[~valid]=0
    return t,normal


def cylinder_hit(origin,directions,a,b,radius):
    origin=np.asarray(origin,dtype=float);d=np.asarray(directions,dtype=float)
    length=float(np.linalg.norm(b-a));axis=(b-a)/length;offset=origin-a
    along=np.sum(offset*axis,axis=-1);ds=d@axis
    po=offset-np.asarray(along)[...,None]*axis;pd=d-ds[:,None]*axis
    qa=np.sum(pd*pd,axis=1);qb=2*np.sum(pd*po,axis=-1);qc=np.sum(po*po,axis=-1)-radius**2
    disc=qb*qb-4*qa*qc;root=np.sqrt(np.maximum(disc,0))
    result=np.full(len(d),np.inf);normal=np.zeros_like(d)
    for sign in (-1,1):
        t=np.divide(-qb+sign*root,2*qa,out=np.full(len(d),np.inf),where=qa>1e-12)
        axial=along+np.where(np.isfinite(t),t,0)*ds
        valid=(disc>=0)&(t>1e-5)&(axial>=0)&(axial<=length)&(t<result)
        n=po+np.where(valid,t,0)[:,None]*pd
        result[valid]=t[valid];normal[valid]=n[valid]/radius
    for end,sign in ((0.,-1.),(length,1.)):
        t=np.divide(end-along,ds,out=np.full(len(d),np.inf),where=np.abs(ds)>1e-12)
        point=po+np.where(np.isfinite(t),t,0)[:,None]*pd
        valid=(t>1e-5)&(np.sum(point*point,axis=1)<=radius**2)&(t<result)
        result[valid]=t[valid];normal[valid]=sign*axis
    return result,normal


def scene_objects():
    objects=[]
    def box(lo,hi,color,material='solid'):objects.append(('box',np.array(lo,float),np.array(hi,float),np.array(color,float),material))
    def tube(a,b,r,color):objects.append(('cylinder',np.array(a,float),np.array(b,float),float(r),np.array(color,float)))
    box((-2.45,-1.7,-.08),(2.45,2.2,0),(100,116,124),'floor')
    box((-1.62,-.48,.95),(.28,.64,1.08),(90,105,110),'bench')
    for x in (-1.5,.04):
        for y in (-.36,.48):box((x,y,0),(x+.12,y+.12,.95),(49,65,73))
    box((-1.49,-.27,.2),(.12,.5,.27),(54,72,81))
    box((.85,.72,0),(1.72,1.49,2.23),(30,79,112),'cabinet')
    box((.96,.675,1.03),(1.60,.721,2.06),(66,104,121),'panel')
    box((1.04,.65,1.47),(1.51,.676,1.87),(26,39,47),'screen')
    box((.97,.66,.16),(1.59,.717,.9),(39,70,89),'vents')
    tube((1.55,.62,.31),(1.55,.62,.76),.03,(159,175,174))
    box((-.54,.03,1.08),(-.08,.44,1.52),(228,154,66),'crate')
    box((-1.40,-.15,1.08),(-1.04,.24,1.34),(112,140,151),'toolbox')
    box((-1.02,.05,1.08),(-.75,.30,1.18),(180,196,194))
    box((.34,-.58,0),(.92,-.05,.46),(190,125,58),'crate')
    box((.40,-.49,.46),(.82,-.13,.79),(220,160,75),'crate')
    tube((.48,1.01,.15),(.48,1.01,1.60),.10,(73,132,139))
    tube((-.02,1.01,1.57),(.84,1.01,1.57),.10,(76,146,151))
    tube((.48,.52,1.45),(.48,1.02,1.45),.072,(101,120,119))
    center=np.array([.48,.50,1.45]);radius=.30
    ring=[center+np.array([radius*np.cos(a),0,radius*np.sin(a)]) for a in np.linspace(0,2*np.pi,17)]
    for a,b in zip(ring,ring[1:]):tube(a,b,.057,(230,64,44))
    tube(center+[-radius,0,0],center+[radius,0,0],.038,(226,58,41))
    tube(center+[0,0,-radius],center+[0,0,radius],.038,(226,58,41))
    tube(center+[0,-.04,0],center+[0,.04,0],.095,(233,75,49))
    return objects


def _intersection(origin,directions,obj):
    return box_hit(origin,directions,obj[1],obj[2]) if obj[0]=='box' else cylinder_hit(origin,directions,obj[1],obj[2],obj[3])


def cast_scene(origin,directions,texture,*,shadows=True):
    objects=scene_objects();nearest=np.full(len(directions),np.inf);normals=np.zeros_like(directions)
    colors=np.zeros_like(directions);which=np.full(len(directions),-1,dtype=int)
    for index,obj in enumerate(objects):
        t,n=_intersection(origin,directions,obj);hit=(t<nearest)&(t<12.)
        nearest[hit]=t[hit];normals[hit]=n[hit];which[hit]=index
    valid=np.isfinite(nearest);world=origin+directions*np.where(valid,nearest,0)[:,None]
    for index,obj in enumerate(objects):
        selected=which==index
        if not np.any(selected):continue
        color=obj[3] if obj[0]=='box' else obj[4];colors[selected]=color
        if obj[0]!='box':continue
        material=obj[4];p=world[selected];n=normals[selected]
        if material=='bench':
            top=n[:,2]>.5
            u=np.clip((p[:,0]-obj[1][0])/(obj[2][0]-obj[1][0]),0,1)
            v=np.clip((obj[2][1]-p[:,1])/(obj[2][1]-obj[1][1]),0,1)
            sample=texture[np.minimum((v*texture.shape[0]).astype(int),texture.shape[0]-1),np.minimum((u*texture.shape[1]).astype(int),texture.shape[1]-1)]
            current=colors[selected];current[top]=sample[top];colors[selected]=current
        elif material=='floor':
            grid=((np.abs((p[:,0]+.5)%1-.5)<.012)|(np.abs((p[:,1]+.5)%1-.5)<.012))
            current=colors[selected];current[grid]*=.70
            # Painted aisle marks are color on a flat floor, not extra geometry.
            stripe=(p[:,1]<-1.1)&(p[:,1]>-1.16);current[stripe]=[194,145,54];colors[selected]=current
        elif material in ('crate','toolbox'):
            edge=np.minimum(np.abs(p-obj[1]),np.abs(obj[2]-p))
            boundary=np.sum(edge<.035,axis=1)>=2
            current=colors[selected];current[boundary]*=.48
            band=np.abs((p[:,0]-obj[1][0])/(obj[2][0]-obj[1][0])-.5)<.05
            current[band]*=.82;colors[selected]=current
        elif material=='vents':
            current=colors[selected];current[(np.sin(p[:,2]*75)>.15)&(n[:,1]<-.5)]*=.48;colors[selected]=current
        elif material=='screen':
            current=colors[selected];line=(np.abs(p[:,2]-1.70)<.018)|(np.abs(p[:,2]-1.58)<.011)
            current[line]=[67,193,163];colors[selected]=current
    light=np.array([-.35,-.5,1.]);light/=np.linalg.norm(light)
    lighting=.64+.36*np.maximum(normals@light,0)
    if shadows:
        blocked=np.zeros(len(directions),dtype=bool);start=world+normals*.001
        rays=np.broadcast_to(light,directions.shape)
        for obj in objects:
            t,_=_intersection(start,rays,obj);blocked|=valid&np.isfinite(t)&(t>1e-4)
        lighting[blocked]*=.68
    colors=np.clip(colors*lighting[:,None],0,255).astype('u1');colors[~valid]=0
    return nearest,colors


def camera_inputs(texture,*,pose_shift=0.,capture_ns=1000000000,camera_ids=tuple(range(6))):
    views=[((-2.7,-3.3,2.25),(-.2,.55,1.05),63),((2.8,-3.0,2.3),(.1,.55,1.05),63),
           ((-.1,-3.2,3.3),(-.1,.50,.95),60),((-1.25,-1.9,2.05),(-.8,.15,1.02),58),
           ((1.35,-1.8,2.15),(.95,.85,1.32),58),((-1.8,1.7,2.7),(-.2,.55,1.1),65)]
    cameras=[];frames=[];inputs=[]
    for camera_id,(position,target,fov) in enumerate(views):
        if camera_id not in camera_ids:continue
        position=np.array(position,float)+pose_shift*np.array([math.sin(camera_id+1),math.cos(camera_id+1),.25])
        pose=camera_pose(position,target)
        def intrinsics(width,height):
            f=width/(2*np.tan(np.deg2rad(fov/2)))
            return Intrinsics(width,height,float(f),float(f),(width-1)/2,(height-1)/2)
        dk,ik=intrinsics(128,96),intrinsics(256,192)
        def rays(k):
            yy,xx=np.mgrid[:k.height,:k.width]
            optical=np.stack(((xx-k.ppx)/k.fx,(yy-k.ppy)/k.fy,np.ones_like(xx)),axis=-1).reshape(-1,3)
            return optical@pose[:3,:3].T
        depth,_=cast_scene(pose[:3,3],rays(dk),texture,shadows=False)
        _,rgb=cast_scene(pose[:3,3],rays(ik),texture,shadows=True)
        depth=np.where(np.isfinite(depth),np.rint(np.minimum(depth,65.535)*1000),0).astype('<u2').reshape(dk.height,dk.width)
        rgb=rgb.reshape(ik.height,ik.width,3)
        camera=CameraCalibration(camera_id,dk,ik,.001,np.eye(4),np.eye(4),image_format='RGB8')
        frame=DepthFrame(camera_id,2 if capture_ns==1000000000 else 1,capture_ns,depth,rgb,world_from_rig=pose)
        cameras.append(camera);frames.append(frame);inputs.append((rgb,depth))
    return cameras,frames,inputs


def build_scene_packets(texture):
    cameras,frames,inputs=camera_inputs(texture)
    _,previous,_=camera_inputs(texture,pose_shift=.18,capture_ns=900000000,camera_ids=(0,1,3,4))
    common=dict(source_id=0x52495645544d4544,rig_mode='posed',observer_pose=OBSERVER,synthetic=True,retention_enabled=True)
    built=[]
    for geometry in ('prepared','unprepared'):
        cache=RetainedObservationMap()
        build_rgb_batch(previous,cameras,geometry='unprepared',sequence=1,produced_ns=901000000,observation_map=cache,**common)
        built.append(build_rgb_batch(frames,cameras,geometry=geometry,sequence=2,produced_ns=1001000000,observation_map=cache,**common)[0])
    prepared,raw=built
    prepared_bytes=pack_packet(prepared);raw_bytes=pack_packet(raw)
    decoded=unpack_packet(prepared_bytes);decoded_raw=prepare_rgb(unpack_packet(raw_bytes))
    equal=all(np.array_equal(getattr(decoded,name),getattr(decoded_raw,name)) for name in ('vertices','indices','atlas'))
    if not equal:raise AssertionError('Prepared and unprepared production paths differ')
    proof=dict(scope='synthetic calibrated RGB-D benchmark; host software preview, not a headset capture',
        input_origin='generated planar workbench albedo plus analytic boxes/cylinders; depth is ray-intersection optical Z',
        camera_configuration='six synthetic RGB-D views, different from the proposed five-Y8 plus one-RGB physical rig',
        cameras=6,current_mask=decoded.contributing_mask,retained_observations=sum(bool(v.flags&1) for v in decoded.views),observation_count=len(decoded.views),raw_protocol_version=3,prepared_unprepared_equal=equal,
        prepared_bytes=len(prepared_bytes),unprepared_bytes=len(raw_bytes),vertices=len(decoded.vertices),indices=len(decoded.indices),
        triangles=len(decoded.indices)//3,atlas_shape=list(decoded.atlas.shape),
        calibrated_views=[dict(camera_id=c.camera_id,depth=asdict(c.depth),image=asdict(c.intensity),depth_units_m=c.depth_units_m,
                               world_from_depth=f.world_from_rig.tolist(),depth_sha256=hashlib.sha256(f.depth.tobytes()).hexdigest(),
                               rgb_sha256=hashlib.sha256(f.intensity.tobytes()).hexdigest()) for c,f in zip(cameras,frames)],
        prepared_sha256=hashlib.sha256(prepared_bytes).hexdigest(),unprepared_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        pipeline='tools.remote_scene.rgb_surface.build_rgb_batch -> protocol.pack_packet/unpack_packet -> decoded mesh',
        physics_used=False,rendering_backend='NumPy/Pillow software z-buffer with projective bilinear atlas sampling',
        observations=[dict(camera_id=v.camera_id,retained=bool(v.flags&1),observation_id=v.observation_id,
                           capture_ns=v.capture_ns,world_from_depth=v.world_from_depth.tolist()) for v in decoded_raw.views])
    return decoded,decoded_raw,inputs,proof


def rasterize(packet,eye,target,width=640,height=360):
    camera=camera_pose(np.asarray(eye,float),target)
    points=(packet.vertices[:,:3].astype(float)-camera[:3,3])@camera[:3,:3]
    f=width/(2*np.tan(np.deg2rad(60)/2));z=points[:,2]
    screen=points[:,:2]*f/z[:,None]+[width*.5,height*.55]
    ys=np.linspace(0,1,height)[:,None,None];background=np.array([13,22,29])[None,None,:]*(1-ys)+np.array([24,37,45])[None,None,:]*ys
    output=np.broadcast_to(background,(height,width,3)).copy().astype('u1');depth=np.full((height,width),np.inf)
    vertices=packet.vertices.astype(float);atlas=packet.atlas;ah,aw=atlas.shape[:2]
    for ids in packet.indices.reshape(-1,3):
        if np.any(z[ids]<=.05):continue
        p=screen[ids];denom=(p[1,1]-p[2,1])*(p[0,0]-p[2,0])+(p[2,0]-p[1,0])*(p[0,1]-p[2,1])
        if denom>=-1e-8:continue
        x0=max(0,int(np.floor(np.min(p[:,0]))));x1=min(width-1,int(np.ceil(np.max(p[:,0]))))
        y0=max(0,int(np.floor(np.min(p[:,1]))));y1=min(height-1,int(np.ceil(np.max(p[:,1]))))
        if x0>x1 or y0>y1:continue
        yy,xx=np.mgrid[y0:y1+1,x0:x1+1];xx=xx+.5;yy=yy+.5
        a=((p[1,1]-p[2,1])*(xx-p[2,0])+(p[2,0]-p[1,0])*(yy-p[2,1]))/denom
        b=((p[2,1]-p[0,1])*(xx-p[2,0])+(p[0,0]-p[2,0])*(yy-p[2,1]))/denom;c=1-a-b
        weights=np.stack((a,b,c),axis=-1);inside=np.all(weights>=-1e-7,axis=-1)
        perspective=weights/z[ids];invz=perspective.sum(axis=-1)
        sample_depth=np.divide(1.,invz,out=np.full_like(invz,np.inf),where=invz>0)
        local=depth[y0:y1+1,x0:x1+1];update=inside&(sample_depth<local)
        if not np.any(update):continue
        uvq=perspective@vertices[ids,3:6]
        q=uvq[...,2];u=np.divide(uvq[...,0],q,out=np.zeros_like(q),where=q>0);v=np.divide(uvq[...,1],q,out=np.zeros_like(q),where=q>0)
        valid=np.divide(perspective@vertices[ids,10],invz,out=np.zeros_like(invz),where=invz>0)
        bounds=vertices[ids[0],6:10]
        textured=(q>0)&(u>=bounds[0])&(u<=bounds[2])&(v>=bounds[1])&(v<=bounds[3])&(valid>=1-1e-6)
        tx=np.clip(u*aw-.5,0,aw-1);ty=np.clip(v*ah-.5,0,ah-1)
        ix=np.floor(tx).astype(int);iy=np.floor(ty).astype(int);fx=(tx-ix)[...,None];fy=(ty-iy)[...,None]
        ix1=np.minimum(ix+1,aw-1);iy1=np.minimum(iy+1,ah-1)
        color=(atlas[iy,ix]*(1-fx)+atlas[iy,ix1]*fx)*(1-fy)+(atlas[iy1,ix]*(1-fx)+atlas[iy1,ix1]*fx)*fy
        color=np.where(textured[...,None],color,128).astype('u1')
        output[y0:y1+1,x0:x1+1][update]=color[update];local[update]=sample_depth[update]
    return Image.fromarray(output)


def font(size,path=None):
    if path:return ImageFont.truetype(str(path),size)
    try:return ImageFont.truetype('DejaVuSans.ttf',size)
    except OSError:return ImageFont.load_default(size=size)


def annotate(frame,proof,index,total,font_path=None):
    draw=ImageDraw.Draw(frame);w,h=frame.size
    draw.rectangle((0,0,w,45),fill=(10,18,25));draw.rectangle((0,h-29,w,h),fill=(10,18,25))
    draw.text((14,8),'RIVET XR',font=font(21,font_path),fill=(238,241,239))
    draw.text((143,15),'REMOTE INSPECTION',font=font(10,font_path),fill=(145,171,176))
    label='SYNTHETIC \u2022 PIPELINE OUTPUT';box=draw.textbbox((0,0),label,font=font(10,font_path))
    draw.text((w-14-(box[2]-box[0]),15),label,font=font(10,font_path),fill=(226,179,99))
    draw.text((14,h-21),f"HOST RENDER  /  6 CAMERAS + 4 RETAINED  /  {proof['triangles']:,} TRIANGLES",font=font(10,font_path),fill=(151,177,181))
    draw.line((14,h-4,14+(w-28)*index/max(1,total-1),h-4),fill=(84,194,167),width=2)
    return frame


def input_sheet(inputs,path,font_path=None):
    cellw,cellh=336,154;sheet=Image.new('RGB',(cellw*3,cellh*2+64),(12,22,29));draw=ImageDraw.Draw(sheet)
    draw.text((14,9),'SYNTHETIC CALIBRATED INPUTS',font=font(20,font_path),fill=(234,240,235))
    draw.text((14,35),'Six RGB-D cameras \u2022 known geometry \u2022 depth from ray intersections',font=font(12,font_path),fill=(146,173,180))
    for i,(rgb,depth) in enumerate(inputs):
        x=(i%3)*cellw+8;y=(i//3)*cellh+62
        draw.text((x,y),f'CAMERA {i}   RGB                         OPTICAL Z',font=font(11,font_path),fill=(160,185,188))
        sheet.paste(Image.fromarray(rgb).resize((156,117),Image.Resampling.LANCZOS),(x,y+20))
        z=depth.astype(float)*.001;t=np.clip((z-.5)/5.5,0,1)
        color=np.stack((255*np.clip(1.7*t-.35,0,1),220*np.sin(t*np.pi*.8),255*(1-t)),axis=-1).astype('u1');color[z==0]=0
        sheet.paste(Image.fromarray(color).resize((156,117),Image.Resampling.NEAREST),(x+164,y+20))
    sheet.save(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--texture',type=Path,required=True,help='Planar workbench albedo only; never used to infer depth')
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--font',type=Path)
    parser.add_argument('--texture-origin',choices=('user-provided','ai-generated'),default='user-provided')
    parser.add_argument('--frames',type=int,default=80);parser.add_argument('--fps',type=int,default=10)
    parser.add_argument('--width',type=int,default=640);parser.add_argument('--height',type=int,default=360)
    parser.add_argument('--supersample',type=int,choices=(1,2),default=2,help='Host raster antialiasing, without changing decoded geometry')
    args=parser.parse_args()
    if not 1<=args.frames<=160 or not 1<=args.fps<=30 or not 320<=args.width<=1280 or not 180<=args.height<=720:parser.error('output dimensions/frame limits exceeded')
    args.output.mkdir(parents=True,exist_ok=True);started=time.perf_counter()
    texture=np.asarray(Image.open(args.texture).convert('RGB'),dtype='u1')
    packet,raw_packet,inputs,proof=build_scene_packets(texture)
    proof['texture_sha256']=hashlib.sha256(args.texture.read_bytes()).hexdigest();proof['texture_role']='provided albedo on known planar workbench; no image-derived depth';proof['texture_origin']=args.texture_origin
    proof['texture_file']=args.texture.name;proof['font_file']=args.font.name if args.font else 'default'
    proof['generator_environment']=dict(python=sys.version.split()[0],numpy=np.__version__,pillow=PIL.__version__)
    (args.output/'inspection-prepared.rscn').write_bytes(pack_packet(packet));(args.output/'inspection-unprepared.rscn').write_bytes(pack_packet(raw_packet))
    input_sheet(inputs,args.output/'inspection-inputs.png',args.font)
    print(json.dumps({k:proof[k] for k in ('cameras','prepared_bytes','unprepared_bytes','vertices','triangles','prepared_unprepared_equal')}),flush=True)
    frames=[]
    for index in range(args.frames):
        angle=np.deg2rad(25)*math.sin(index/args.frames*2*math.pi)
        target=np.array([-.05,.35,.95]);eye=target+np.array([5.2*math.sin(angle),-5.2*math.cos(angle),2.35])
        frame=rasterize(packet,eye,target,args.width*args.supersample,args.height*args.supersample)
        if args.supersample>1:frame=frame.resize((args.width,args.height),Image.Resampling.LANCZOS)
        frames.append(annotate(frame,proof,index,args.frames,args.font))
        if index%10==0:print(f'rendered {index+1}/{args.frames}',flush=True)
    frames[0].save(args.output/'inspection-poster.png')
    palette_source=Image.new('RGB',(args.width*3,args.height))
    for column,index in enumerate((0,len(frames)//3,2*len(frames)//3)):palette_source.paste(frames[index],(column*args.width,0))
    palette=palette_source.quantize(colors=192,method=Image.Quantize.MEDIANCUT)
    quantized=[frame.quantize(palette=palette,dither=Image.Dither.NONE) for frame in frames]
    quantized[0].save(args.output/'inspection-orbit.gif',save_all=True,append_images=quantized[1:],duration=1000/args.fps,loop=0,disposal=2,optimize=True)
    with Image.open(args.output/'inspection-orbit.gif') as gif:
        sheet=Image.new('RGB',(args.width*2,args.height),(10,18,25))
        for cell in range(8):
            gif.seek(min(gif.n_frames-1,cell*gif.n_frames//8))
            tile=gif.convert('RGB').resize((args.width//2,args.height//2),Image.Resampling.LANCZOS)
            sheet.paste(tile,((cell%4)*(args.width//2),(cell//4)*(args.height//2)))
        sheet.save(args.output/'inspection-orbit-contact-sheet.png')
    proof.update(frame_count=args.frames,fps=args.fps,width=args.width,height=args.height,supersample=args.supersample,host_generation_seconds=time.perf_counter()-started,
                 output_kind='host software rasterization of decoded production RSCN v3 mesh',has_headset_capture=False)
    (args.output/'provenance.json').write_text(json.dumps(proof,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(output_gif=str(args.output/'inspection-orbit.gif'),host_seconds=proof['host_generation_seconds'])),flush=True)

if __name__=='__main__':main()
