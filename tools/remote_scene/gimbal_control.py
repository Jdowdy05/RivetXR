"""Simulation-only gimbal intent with receive-clock challenges and short leases."""
from dataclasses import dataclass
import math
import secrets
import struct

COMMAND=struct.Struct('<4sHH4Q2I2f2I')
FEEDBACK=struct.Struct('<4sHH4Q2fQ2IQ5fI')
HOLD,AIM,RESET_MAP=0,1,2
SIMULATED,ARMED,TRACKED,LEASE_VALID=1,2,4,8
MAX_LEASE_MS=200
CHALLENGE_NS=250_000_000


@dataclass(frozen=True)
class Command:
    session:int
    challenge:int
    sequence:int
    client_ns:int
    kind:int=HOLD
    clutch:bool=False
    pan:float=0.
    tilt:float=0.
    lease_ms:int=100


def pack_command(value):
    for name in ('session','challenge','sequence','client_ns'):
        item=getattr(value,name)
        if type(item) is not int or not 1<=item<=2**64-1:raise ValueError('invalid gimbal identity/sequence/time')
    if value.kind not in (HOLD,AIM,RESET_MAP) or type(value.clutch) is not bool:
        raise ValueError('invalid gimbal operation')
    if value.kind!=AIM and value.clutch:raise ValueError('clutch only applies to aim')
    if type(value.lease_ms) is not int or not 50<=value.lease_ms<=MAX_LEASE_MS:raise ValueError('invalid gimbal lease')
    if any(not math.isfinite(v) for v in (value.pan,value.tilt)):raise ValueError('nonfinite gimbal target')
    return COMMAND.pack(b'QGCM',1,64,value.session,value.challenge,value.sequence,value.client_ns,
                        value.kind,int(value.clutch),value.pan,value.tilt,value.lease_ms,0)


def unpack_command(raw):
    if len(raw)!=COMMAND.size:raise ValueError('invalid gimbal command size')
    h=COMMAND.unpack(raw)
    if h[:3]!=(b'QGCM',1,64) or h[-1] or h[8] not in (0,1):raise ValueError('invalid gimbal command header/flags')
    value=Command(*h[3:8],bool(h[8]),h[9],h[10],h[11])
    pack_command(value)
    return value


def unpack_feedback(raw):
    if len(raw)!=96:raise ValueError('invalid gimbal feedback size')
    h=FEEDBACK.unpack(raw)
    if h[:3]!=(b'QGCF',1,96) or h[-1] or not h[3] or not h[4] or not h[6]:
        raise ValueError('invalid gimbal feedback header')
    if h[10]&~15 or not h[10]&SIMULATED or not h[12] or not h[9]:raise ValueError('only simulated control with a valid scene/map is supported')
    if not all(math.isfinite(v) for v in (*h[7:9],*h[13:18])):raise ValueError('nonfinite gimbal feedback')
    if not h[13]<h[14] or not h[15]<h[16] or h[17]<=0 or h[11]>MAX_LEASE_MS:
        raise ValueError('invalid gimbal feedback limits')
    return dict(session=h[3],challenge=h[4],ack=h[5],server_ns=h[6],pan=h[7],tilt=h[8],
                source_id=h[9],flags=h[10],lease_ms=h[11],map_generation=h[12],
                pan_min=h[13],pan_max=h[14],tilt_min=h[15],tilt_max=h[16],max_rate=h[17])


class GimbalSession:
    """One owner calls under its service lock; no physical adapter is accepted."""
    def __init__(self,driver,now_ns,*,reset_map=lambda:None,map_generation=lambda:1,source_id=1,nonce=None):
        if getattr(driver,'simulation_only',False) is not True:raise ValueError('physical gimbal drivers are not enabled')
        if type(source_id) is not int or not 1<=source_id<=2**64-1:raise ValueError('invalid paired scene source')
        self.driver=driver;self.reset_map=reset_map;self.map_generation=map_generation;self.source_id=source_id
        self.nonce=nonce or (secrets.randbits(64) or 1)
        self.challenge=secrets.randbits(64) or 1;self.challenge_issued=now_ns
        self.sequence=0;self.client_ns=0;self.expiry=0;self.armed=False;self.needs_release=True
        self.driver.hold(now_ns)

    def tick(self,now_ns):
        if self.armed and now_ns>=self.expiry:
            self.driver.advance(self.expiry);self.driver.hold(self.expiry)
            self.armed=False;self.needs_release=True
        self.driver.advance(now_ns)

    def close(self,now_ns):
        self.tick(now_ns);self.driver.hold(now_ns);self.armed=False;self.needs_release=True

    def reply(self,now_ns,*,new_challenge=False):
        self.tick(now_ns)
        if new_challenge:
            self.challenge=secrets.randbits(64) or 1;self.challenge_issued=now_ns
        f=self.driver.feedback();limits=self.driver.limits
        flags=SIMULATED|(TRACKED if f.tracked else 0)|(ARMED|LEASE_VALID if self.armed else 0)
        remaining=max(0,min(MAX_LEASE_MS,(self.expiry-now_ns)//1_000_000)) if self.armed else 0
        generation=int(self.map_generation())
        if not 1<=generation<=2**64-1:raise ValueError('invalid map generation')
        return FEEDBACK.pack(b'QGCF',1,96,self.nonce,self.challenge,self.sequence,now_ns,
            f.pan_rad,f.tilt_rad,self.source_id,flags,remaining,generation,
            limits.pan_min_rad,limits.pan_max_rad,limits.tilt_min_rad,limits.tilt_max_rad,limits.max_rate_rad_s,0)

    def handle(self,raw,now_ns):
        self.tick(now_ns)
        try:c=unpack_command(raw)
        except (ValueError,TypeError,struct.error):
            self.close(now_ns);raise
        if c.session!=self.nonce or c.sequence<=self.sequence or c.client_ns<=self.client_ns:
            self.close(now_ns);raise ValueError('gimbal command session/replay rejection')
        self.sequence,self.client_ns=c.sequence,c.client_ns
        challenge_valid=c.challenge==self.challenge and now_ns-self.challenge_issued<=CHALLENGE_NS
        if c.kind==HOLD or not c.clutch and c.kind==AIM:
            self.driver.hold(now_ns);self.armed=False;self.needs_release=False
        elif not challenge_valid:
            self.driver.hold(now_ns);self.armed=False;self.needs_release=True
        elif c.kind==RESET_MAP:
            self.driver.hold(now_ns);self.armed=False;self.needs_release=True;self.reset_map()
        elif not self.needs_release:
            limits=self.driver.limits
            f32=lambda x:struct.unpack('<f',struct.pack('<f',x))[0]
            if not f32(limits.pan_min_rad)<=c.pan<=f32(limits.pan_max_rad) or not f32(limits.tilt_min_rad)<=c.tilt<=f32(limits.tilt_max_rad):
                self.close(now_ns);raise ValueError('gimbal target outside simulated limits')
            pan=min(limits.pan_max_rad,max(limits.pan_min_rad,c.pan))
            tilt=min(limits.tilt_max_rad,max(limits.tilt_min_rad,c.tilt))
            self.driver.target(pan,tilt,now_ns);self.expiry=now_ns+c.lease_ms*1_000_000;self.armed=True
        return self.reply(now_ns,new_challenge=True)
