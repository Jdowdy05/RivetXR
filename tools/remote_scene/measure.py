"""Bounded synthetic host timing; no RealSense, network, or Quest performance claim."""
import argparse
import json
from pathlib import Path
import platform
import time
import zlib
import numpy as np
from .demo import synthetic_batch,OBSERVER
from .fusion import fuse_batch
from .protocol import pack_packet,integer


def measure(iterations=5,width=128,height=96):
    integer(iterations,1,20,'measurement iterations')
    samples=[]
    for _ in range(iterations):
        start=time.perf_counter();cameras,frames=synthetic_batch(width,height);generated=time.perf_counter()
        packet,stats=fuse_batch(frames,cameras,produced_ns=1005000000,rig_mode='stationary',observer_pose=OBSERVER,synthetic=True)
        fused=time.perf_counter();raw=pack_packet(packet);packed=time.perf_counter();compressed=zlib.compress(raw,1);ended=time.perf_counter()
        samples.append(dict(generate_ms=(generated-start)*1000,fuse_ms=(fused-generated)*1000,
                            pack_ms=(packed-fused)*1000,zlib_ms=(ended-packed)*1000,total_ms=(ended-start)*1000))
    summary={key:dict(mean=float(np.mean([s[key] for s in samples])),p95=float(np.percentile([s[key] for s in samples],95)),
                      maximum=max(s[key] for s in samples)) for key in samples[0]}
    return dict(scope='synthetic host only; no camera/Quest/LAN throughput claim',python=platform.python_version(),numpy=np.__version__,
                width=width,height=height,cameras=5,iterations=iterations,statistics=stats,raw_bytes=len(raw),zlib_bytes=len(compressed),
                hypothetical_10hz_wire_mbps=(len(compressed)+8)*10*8/1000000,samples_ms=samples,summary_ms=summary)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path)
    parser.add_argument('--iterations',type=int,default=5);parser.add_argument('--width',type=int,default=128);parser.add_argument('--height',type=int,default=96)
    args=parser.parse_args();result=measure(args.iterations,args.width,args.height);raw=json.dumps(result,indent=2)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(raw,encoding='utf-8')
    print(raw)


if __name__=='__main__':main()
