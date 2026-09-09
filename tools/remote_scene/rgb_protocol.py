"""RSCN v3 RGB observation framing. V1/v2 wire contracts remain separate."""
from dataclasses import dataclass
import struct
import zlib
import numpy as np
from .protocol import HEADER,integer,validate_common
from .surface_protocol import SurfacePacket,SurfaceView,EXTENSION,VIEW_HEADER,readonly,_intrinsics,_transform

V3_EXTRA=struct.Struct('<QQ8I2Q')
RAW_EXTRA=struct.Struct('<QQIIIIQQ')
META=struct.Struct('<IIQQQ10IQ')
MAX_VERTICES,MAX_INDICES,MAX_PIXELS=32768,196608,768000
MAX_BYTES=4*1024*1024

@dataclass(frozen=True)
class RGBView(SurfaceView):
    image_capture_ns:int=0
    observation_id:int=0
    flags:int=2
    image_format:int=1
    triangle_keep:bytes=b''

@dataclass(frozen=True)
class RGBMeta:
    camera_id:int
    flags:int
    observation_id:int
    capture_ns:int
    image_capture_ns:int
    vertex_start:int
    vertex_count:int
    index_start:int
    index_count:int
    tile_x:int
    tile_y:int
    tile_width:int
    tile_height:int
    image_format:int

@dataclass(frozen=True)
class RGBPacket(SurfacePacket):
    map_generation:int=1
    retention_ms:int=30000
    map_flags:int=1
    prepared_views:tuple=()
    @property
    def protocol_version(self):return 3
    @property
    def identity(self):return self.source_id,self.world_epoch,self.calibration_id,self.representation,self.map_generation
    @property
    def oldest_observation_ns(self):return min((observation_stamp(v) for v in self.views),default=0)
    @property
    def geometry_summary(self):
        return dict(representation=self.representation,vertices=len(self.vertices),triangles=len(self.indices)//3,
                    atlas_width=self.atlas.shape[1],atlas_height=self.atlas.shape[0],views=len(self.views),
                    retained_views=sum(bool(v.flags&1) for v in self.views),map_generation=self.map_generation,
                    oldest_observation_ns=self.oldest_observation_ns)


def observation_stamp(view):
    return min(view.capture_ns,view.image_capture_ns) if view.flags&2 else view.capture_ns

def mask_size(w,h):return (2*(w-1)*(h-1)+7)//8

def full_mask(w,h):return np.packbits(np.ones(2*(w-1)*(h-1),dtype='u1'),bitorder='little').tobytes()

def shelf(dimensions):
    tiles=[];y=0;width=0
    for start in range(0,len(dimensions),5):
        row=dimensions[start:start+5];x=0;rowheight=max((h for w,h in row),default=0)
        for w,h in row:tiles.append((x,y,w,h));x+=w
        width=max(width,x);y+=rowheight
    return width,y,tiles


def _validate_times(packet):
    integer(packet.map_generation,1,2**64-1,'map generation');integer(packet.retention_ms,1,60000,'retention age')
    integer(packet.map_flags,0,3,'map flags')
    if len(packet.views)>10:raise ValueError('too many observations')
    current=[];retained=[];keys=set()
    for view in packet.views:
        integer(view.camera_id,0,5,'camera ID');integer(view.flags,0,3,'observation flags')
        integer(view.observation_id,1,2**64-1,'observation ID');integer(view.image_format,1,3,'image format')
        if view.image_format not in (1,3):raise ValueError('invalid image format')
        integer(view.capture_ns,1,packet.produced_ns,'depth exposure');integer(view.image_capture_ns,1,packet.produced_ns,'image exposure')
        key=(view.camera_id,view.observation_id)
        if key in keys:raise ValueError('duplicate observation')
        keys.add(key)
        if view.flags&1:
            retained.append(key)
            if packet.produced_ns-observation_stamp(view)>packet.retention_ms*1000000:raise ValueError('expired retained observation')
        else:
            current.append(view.camera_id)
            if not packet.capture_start_ns<=view.capture_ns<=packet.capture_end_ns:raise ValueError('current depth exposure outside interval')
            if view.flags&2 and packet.produced_ns-view.image_capture_ns>2000000000:raise ValueError('stale current image')
    if len(current)>6 or len(retained)>4 or current!=sorted(set(current)) or retained!=sorted(retained):raise ValueError('invalid observation ordering/count')
    if [bool(v.flags&1) for v in packet.views]!=[False]*len(current)+[True]*len(retained):raise ValueError('current observations must precede retained')
    if sum(1<<i for i in current)!=packet.contributing_mask:raise ValueError('current mask mismatch')
    if retained and not packet.map_flags&2:raise ValueError('retention disabled but old observations supplied')
    if not packet.map_flags&1 and (packet.views or packet.contributing_mask or len(packet.vertices)):raise ValueError('tracking-invalid map contains observations')


def _validate_raw(view):
    d=view.depth;i=view.intensity
    if not isinstance(d,np.ndarray) or d.dtype!=np.dtype('<u2') or d.ndim!=2 or not 2<=d.shape[1]<=64 or not 2<=d.shape[0]<=48:raise ValueError('invalid RGB depth image')
    if not isinstance(i,np.ndarray) or i.dtype!=np.dtype('u1') or i.ndim!=(3 if view.image_format==3 else 2) or not 2<=i.shape[1]<=256 or not 2<=i.shape[0]<=192 or (view.image_format==3 and i.shape[2]!=3):raise ValueError('invalid RGB image')
    _intrinsics(view.depth_intrinsics,'depth');_intrinsics(view.intensity_intrinsics,'image')
    _transform(view.world_from_depth);_transform(view.intensity_from_depth)
    if not np.isfinite(view.depth_units_m) or not 0<float(np.float32(view.depth_units_m))<=1:raise ValueError('invalid depth units')
    count=2*(d.shape[1]-1)*(d.shape[0]-1)
    if not isinstance(view.triangle_keep,bytes) or len(view.triangle_keep)!=mask_size(d.shape[1],d.shape[0]):raise ValueError('invalid triangle mask size')
    if count%8 and view.triangle_keep[-1]>>count%8:raise ValueError('nonzero triangle mask padding')


def validate_rgb(packet):
    validate_common(packet,maximum_mask=63)
    integer(packet.representation,1,2,'RGB representation');_validate_times(packet)
    v=packet.vertices;i=packet.indices;a=packet.atlas
    if not isinstance(v,np.ndarray) or v.dtype!=np.dtype('<f4') or v.ndim!=2 or v.shape[1]!=11 or len(v)>MAX_VERTICES:raise ValueError('invalid RGB vertices')
    if not isinstance(i,np.ndarray) or i.dtype!=np.dtype('<u2') or i.ndim!=1 or len(i)>MAX_INDICES or len(i)%3:raise ValueError('invalid RGB indices')
    if not isinstance(a,np.ndarray) or a.dtype!=np.dtype('u1') or a.ndim!=3 or a.shape[2]!=3 or max(a.shape[:2])>2048 or a.shape[0]*a.shape[1]>MAX_PIXELS:raise ValueError('invalid RGB atlas')
    if not np.isfinite(v).all() or np.any(np.abs(v[:,:3])>100) or np.any((v[:,10]!=0)&(v[:,10]!=1)):raise ValueError('invalid RGB vertex values')
    if np.any(i>=len(v)):raise ValueError('RGB index out of range')
    if len(i):
        triangles=i.reshape(-1,3);p=v[triangles,:3].astype(float)
        cross=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0])
        if np.any(np.sum(cross*cross,axis=1)<=1e-12):raise ValueError('degenerate RGB triangles')
    if packet.representation==2:
        for view in packet.views:
            if not isinstance(view,RGBView):raise ValueError('raw RGB view required')
            _validate_raw(view)
        # A locally prepared raw packet can retain derived immutable geometry.
        return
    vertex_start=index_start=0;dimensions=[]
    for meta in packet.views:
        if not isinstance(meta,RGBMeta):raise ValueError('prepared RGB metadata required')
        for name in ('vertex_start','vertex_count','index_start','index_count','tile_x','tile_y','tile_width','tile_height'):
            integer(getattr(meta,name),0,2**32-1,name)
        if not 2<=meta.tile_width<=256 or not 2<=meta.tile_height<=192:raise ValueError('invalid RGB tile')
        if meta.vertex_start!=vertex_start or meta.index_start!=index_start or meta.index_count%3:raise ValueError('nonpartitioning observation ranges')
        vertex_start+=meta.vertex_count;index_start+=meta.index_count
        if vertex_start>len(v) or index_start>len(i):raise ValueError('observation range outside arrays')
        local=i[meta.index_start:index_start]
        if np.any(local<meta.vertex_start) or np.any(local>=vertex_start):raise ValueError('triangle crosses observation range')
        dimensions.append((meta.tile_width,meta.tile_height))
        if meta.vertex_count:
            bounds=np.asarray(((meta.tile_x+.5)/a.shape[1],(meta.tile_y+.5)/a.shape[0],
                               (meta.tile_x+meta.tile_width-.5)/a.shape[1],(meta.tile_y+meta.tile_height-.5)/a.shape[0]),dtype='<f4')
            if not np.all(v[meta.vertex_start:vertex_start,6:10]==bounds):raise ValueError('vertex tile bounds differ from observation')
            if not meta.flags&2 and np.any(v[meta.vertex_start:vertex_start,10]):raise ValueError('unusable image marked textured')
    if vertex_start!=len(v) or index_start!=len(i):raise ValueError('observation ranges do not cover arrays')
    width,height,tiles=shelf(dimensions)
    if a.shape!=(height,width,3) or any((m.tile_x,m.tile_y,m.tile_width,m.tile_height)!=tile for m,tile in zip(packet.views,tiles)):raise ValueError('RGB atlas shelf mismatch')


def pack_rgb(packet):
    validate_rgb(packet);pieces=[]
    if packet.representation==1:
        for v in packet.views:pieces.append(META.pack(v.camera_id,v.flags,v.observation_id,v.capture_ns,v.image_capture_ns,
            v.vertex_start,v.vertex_count,v.index_start,v.index_count,v.tile_x,v.tile_y,v.tile_width,v.tile_height,v.image_format,0,0))
        pieces.extend((packet.vertices.tobytes(),packet.indices.tobytes(),packet.atlas.tobytes()))
        nv,ni=len(packet.vertices),len(packet.indices);height,width=packet.atlas.shape[:2]
    else:
        nv=ni=width=height=0
        for v in packet.views:
            dh,dw=v.depth.shape;ih,iw=v.intensity.shape[:2]
            pieces.append(VIEW_HEADER.pack(v.camera_id,dw,dh,iw,ih,0,*v.depth_intrinsics,*v.intensity_intrinsics,v.depth_units_m,
                *np.asarray(v.world_from_depth).reshape(-1),*np.asarray(v.intensity_from_depth).reshape(-1),0,v.capture_ns))
            pieces.append(RAW_EXTRA.pack(v.image_capture_ns,v.observation_id,v.flags,v.image_format,len(v.triangle_keep),0,0,0))
            pieces.extend((v.depth.tobytes(),v.intensity.tobytes(),v.triangle_keep))
    payload=b''.join(pieces);current=sum(not v.flags&1 for v in packet.views);retained=len(packet.views)-current
    if len(payload)+224>MAX_BYTES:raise ValueError('RGB packet exceeds transport budget')
    header=HEADER.pack(b'RSCN',3,224,packet.source_id,packet.world_epoch,packet.sequence,packet.calibration_id,packet.capture_start_ns,
        packet.capture_end_ns,packet.produced_ns,0,packet.expected_mask,packet.contributing_mask,packet.flags,0.,packet.max_capture_skew_ms,
        zlib.crc32(payload),0,*packet.observer_pose,0)
    return header+EXTENSION.pack(packet.representation,nv,ni,width,height,len(packet.views),len(payload),0)+V3_EXTRA.pack(
        packet.map_generation,packet.oldest_observation_ns,current,retained,packet.retention_ms,packet.map_flags,44,3,80 if packet.representation==1 else 208,0,0,0)+payload


def unpack_rgb(raw):
    if not 224<=len(raw)<=MAX_BYTES:raise ValueError('invalid RGB packet size')
    h=HEADER.unpack_from(raw);e=EXTENSION.unpack_from(raw,128);extra=V3_EXTRA.unpack_from(raw,160)
    rep,nv,ni,width,height,nviews,size,reserved=e
    if h[:3]!=(b'RSCN',3,224) or h[10] or struct.unpack_from('<I',raw,80)[0] or h[17] or h[25] or reserved:raise ValueError('invalid RGB common header')
    if rep not in (1,2) or size!=len(raw)-224 or nviews>10 or extra[2]+extra[3]!=nviews or extra[2]>6 or extra[3]>4:raise ValueError('invalid RGB extension')
    if extra[6:9]!=(44,3,80 if rep==1 else 208) or any(extra[9:]):raise ValueError('invalid RGB strides/reserved')
    if zlib.crc32(raw[224:])!=h[16]:raise ValueError('RGB payload CRC mismatch')
    views=[]
    if rep==1:
        if nv>MAX_VERTICES or ni>MAX_INDICES or ni%3 or width>2048 or height>2048 or width*height>MAX_PIXELS or nviews*80+nv*44+ni*2+width*height*3!=size:raise ValueError('invalid RGB prepared lengths')
        offset=224
        for _ in range(nviews):
            m=META.unpack_from(raw,offset);offset+=80
            if m[-2] or m[-1]:raise ValueError('reserved RGB metadata')
            views.append(RGBMeta(*m[:-2]))
        payload=bytes(raw[offset:]);v=np.frombuffer(payload,dtype='<f4',count=nv*11).reshape(nv,11)
        i=np.frombuffer(payload,dtype='<u2',count=ni,offset=nv*44)
        a=np.frombuffer(payload,dtype='u1',count=width*height*3,offset=nv*44+ni*2).reshape(height,width,3)
    else:
        if nv or ni or width or height:raise ValueError('raw RGB declares prepared arrays')
        offset=224;parts=[]
        for _ in range(nviews):
            if offset+208>len(raw):raise ValueError('truncated RGB descriptor')
            d=VIEW_HEADER.unpack_from(raw,offset);x=RAW_EXTRA.unpack_from(raw,offset+160);offset+=208
            camera,dw,dh,iw,ih,res=d[:6];image_capture,obs,flags,fmt,masks,zero,z1,z2=x
            if res or d[39] or zero or z1 or z2 or not 0<=camera<=5 or not 2<=dw<=64 or not 2<=dh<=48 or not 2<=iw<=256 or not 2<=ih<=192 or fmt not in (1,3) or masks!=mask_size(dw,dh):raise ValueError('invalid RGB descriptor')
            nbytes=2*dw*dh+iw*ih*fmt+masks
            if offset+nbytes>len(raw):raise ValueError('truncated RGB image/mask')
            parts.append((d,x,offset,nbytes));offset+=nbytes
        if offset!=len(raw):raise ValueError('trailing RGB payload')
        for d,x,offset,nbytes in parts:
            camera,dw,dh,iw,ih=d[:5];image_capture,obs,flags,fmt,masks=x[:5];payload=bytes(raw[offset:offset+nbytes])
            depth=np.frombuffer(payload,dtype='<u2',count=dw*dh).reshape(dh,dw)
            shape=(ih,iw,3) if fmt==3 else (ih,iw)
            image=np.frombuffer(payload,dtype='u1',count=iw*ih*fmt,offset=2*dw*dh).reshape(shape)
            views.append(RGBView(camera,tuple(d[6:10]),tuple(d[10:14]),d[14],readonly(np.array(d[15:27]).reshape(3,4),'<f4'),
                readonly(np.array(d[27:39]).reshape(3,4),'<f4'),d[40],depth,image,image_capture,obs,flags,fmt,payload[-masks:]))
        v=readonly(np.empty((0,11)),'<f4');i=readonly([],'<u2');a=readonly(np.empty((0,0,3)),'u1')
    packet=RGBPacket(*h[3:10],h[11],h[12],h[13],h[15],tuple(h[18:25]),rep,v,i,a,tuple(views),extra[0],extra[4],extra[5])
    validate_rgb(packet)
    if packet.oldest_observation_ns!=extra[1] or sum(not v.flags&1 for v in views)!=extra[2]:raise ValueError('RGB age/current metadata mismatch')
    return packet
