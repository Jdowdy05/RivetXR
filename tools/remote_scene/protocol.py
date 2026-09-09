"""Versioned RSCN framing and bounded zlib transport, independent of hardware."""
from dataclasses import dataclass
import math
import struct
import zlib
import numpy as np

HEADER=struct.Struct('<4sHH7Q4IfIII7fI')
POINT_DTYPE=np.dtype([('xyz','<f4',(3,)),('gray','u1'),('support','u1'),('radius_mm','<u2')])
MAX_POINTS=50000
MAX_RAW=4*1024*1024
MAX_MESSAGE=4*1024*1024
TRUNCATED,PARTIAL,RECORDED,SYNTHETIC=1,2,4,8


def integer(value,low,high,name):
    if type(value) is not int or not low<=value<=high:raise ValueError(f'invalid {name}')
    return value


@dataclass(frozen=True)
class Packet:
    source_id:int
    world_epoch:int
    sequence:int
    calibration_id:int
    capture_start_ns:int
    capture_end_ns:int
    produced_ns:int
    expected_mask:int
    contributing_mask:int
    flags:int
    voxel_size_m:float
    max_capture_skew_ms:int
    observer_pose:tuple
    points:np.ndarray

    @property
    def identity(self):return self.source_id,self.world_epoch,self.calibration_id,self.representation,0

    @property
    def representation(self):return 0

    @property
    def geometry_summary(self):return dict(representation=0,points=len(self.points))

    @property
    def has_geometry(self):return bool(len(self.points))


def validate_common(packet,maximum_mask=31):
    for name in ('source_id','world_epoch','sequence','calibration_id','produced_ns'):
        integer(getattr(packet,name),1,2**64-1,name)
    for name in ('capture_start_ns','capture_end_ns'):integer(getattr(packet,name),0,2**64-1,name)
    integer(packet.expected_mask,1,maximum_mask,'expected mask');integer(packet.contributing_mask,0,maximum_mask,'contributing mask')
    integer(packet.flags,0,15,'flags');integer(packet.max_capture_skew_ms,1,100,'capture skew')
    if packet.contributing_mask&~packet.expected_mask:raise ValueError('unexpected contributing camera')
    if bool(packet.flags&PARTIAL)!=(packet.contributing_mask!=packet.expected_mask):raise ValueError('partial flag disagrees with masks')
    # Validate what the receiver will actually read after float32 serialization.
    pose=np.asarray(packet.observer_pose,dtype=np.float32).astype(np.float64)
    if pose.shape!=(7,) or not np.isfinite(pose).all() or np.any(np.abs(pose[:3])>100) or abs(np.linalg.norm(pose[3:])-1)>1e-3:
        raise ValueError('invalid observer pose')
    if not packet.contributing_mask:
        if packet.capture_start_ns or packet.capture_end_ns:raise ValueError('empty camera set contains capture data')
    elif not (0<packet.capture_start_ns<=packet.capture_end_ns<=packet.produced_ns
              and packet.capture_end_ns-packet.capture_start_ns<=packet.max_capture_skew_ms*1000000
              and packet.produced_ns-packet.capture_end_ns<=2000000000):
        raise ValueError('invalid capture interval or production age')


def validate(packet):
    if getattr(packet,'protocol_version',0)==3:
        from .rgb_protocol import validate_rgb
        validate_rgb(packet);return
    if not isinstance(packet,Packet):
        from .surface_protocol import validate_surface
        validate_surface(packet);return
    validate_common(packet)
    if not isinstance(packet.voxel_size_m,(int,float,np.floating)) or not math.isfinite(packet.voxel_size_m):raise ValueError('invalid voxel size')
    voxel=float(np.float32(packet.voxel_size_m))
    if not float(np.float32(.001))<=voxel<=float(np.float32(.1)):raise ValueError('voxel size outside limits')
    points=packet.points
    if not isinstance(points,np.ndarray) or points.dtype!=POINT_DTYPE or points.ndim!=1 or len(points)>MAX_POINTS:raise ValueError('invalid point array')
    if not np.isfinite(points['xyz']).all() or np.any(np.abs(points['xyz'])>100):raise ValueError('invalid point position')
    if np.any(points['support']<1) or np.any(points['support']>packet.contributing_mask.bit_count()):raise ValueError('invalid camera support')
    if np.any(points['radius_mm']<1) or np.any(points['radius_mm']>50):raise ValueError('invalid point radius')
    if not packet.contributing_mask and len(points):raise ValueError('empty camera set contains points')


def pack_packet(packet):
    if getattr(packet,'protocol_version',0)==3:
        from .rgb_protocol import pack_rgb
        return pack_rgb(packet)
    if not isinstance(packet,Packet):
        from .surface_protocol import pack_surface
        return pack_surface(packet)
    validate(packet)
    payload=packet.points.tobytes(order='C')
    return HEADER.pack(b'RSCN',1,128,packet.source_id,packet.world_epoch,packet.sequence,packet.calibration_id,
        packet.capture_start_ns,packet.capture_end_ns,packet.produced_ns,len(packet.points),packet.expected_mask,
        packet.contributing_mask,packet.flags,packet.voxel_size_m,packet.max_capture_skew_ms,zlib.crc32(payload),0,
        *packet.observer_pose,0)+payload


def unpack_packet(raw):
    if not isinstance(raw,(bytes,bytearray,memoryview)) or not 128<=len(raw)<=MAX_RAW:raise ValueError('invalid RSCN length')
    h=HEADER.unpack_from(raw)
    if h[1]==3:
        from .rgb_protocol import unpack_rgb
        return unpack_rgb(raw)
    if h[1]==2:
        from .surface_protocol import unpack_surface
        return unpack_surface(raw)
    if h[:3]!=(b'RSCN',1,128) or h[17] or h[25]:raise ValueError('invalid RSCN header or reserved fields')
    count=h[10]
    if count>MAX_POINTS or len(raw)!=128+16*count:raise ValueError('invalid RSCN point count/length')
    payload=bytes(raw[128:])
    if zlib.crc32(payload)!=h[16]:raise ValueError('RSCN payload CRC mismatch')
    points=np.frombuffer(payload,dtype=POINT_DTYPE)
    packet=Packet(*h[3:10],h[11],h[12],h[13],h[14],h[15],tuple(h[18:25]),points)
    validate(packet);return packet


def encode_transport(raw):
    unpack_packet(raw)
    compressed=zlib.compress(raw,level=1)
    if len(raw)>MAX_MESSAGE or len(compressed)>(MAX_MESSAGE if raw[4:6]==b'\x03\x00' else 1024*1024):raise ValueError('transport message too large')
    return struct.pack('!II',len(raw),len(compressed))+compressed


def decode_transport(raw_length,compressed):
    integer(raw_length,128,MAX_MESSAGE,'raw transport size')
    if not 0<len(compressed)<=MAX_MESSAGE:raise ValueError('invalid compressed length')
    decoder=zlib.decompressobj()
    try:raw=decoder.decompress(compressed,raw_length+1)
    except zlib.error as error:raise ValueError('invalid zlib stream') from error
    if len(raw)!=raw_length or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError('zlib length, termination or trailing data mismatch')
    unpack_packet(raw)
    if raw[4:6]!=b'\x03\x00' and len(compressed)>1024*1024:raise ValueError('legacy compressed message too large')
    return raw


class SequenceHistory:
    """At most 64 identities per source session, matching the native mailbox.

    Call accept only after complete packet validation. Callers serialize access;
    no geometry is retained here. Rejection does not change sequence history.
    """
    def __init__(self):self.sequences={}

    def accept(self,packet):
        previous=self.sequences.get(packet.identity)
        if previous is not None and packet.sequence<=previous:
            raise ValueError('duplicate or reversed snapshot sequence within identity')
        if previous is None and len(self.sequences)>=64:
            raise ValueError('identity history is full; restart the source session')
        self.sequences[packet.identity]=packet.sequence


class LatestPacket:
    """Latest validated packet with bounded replay protection across A->B->A."""
    def __init__(self):self.reset()

    def reset(self):
        """Explicitly retire the previous source session, frame and history."""
        self.packet=None;self.history=SequenceHistory()

    def accept(self,raw):
        packet=unpack_packet(raw)
        self.history.accept(packet)
        self.packet=packet
        return packet
