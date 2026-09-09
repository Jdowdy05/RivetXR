"""Run one bounded scene stream; optional gimbal control is simulation-only."""
import argparse
from contextlib import ExitStack
from dataclasses import replace
import hmac
import json
import math
from pathlib import Path
import secrets
import time
from .demo import synthetic_batch,OBSERVER
from .protocol import integer,pack_packet
from .publisher import LatestPublisher
from .recording import Recording,load_json,fields,MAX_BATCHES
from .modes import GEOMETRY_MODES,build_batch


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',choices=('demo','gimbal-demo','replay','realsense'),required=True)
    parser.add_argument('--server-config',required=True,type=Path)
    parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,default=7443)
    parser.add_argument('--manifest',type=Path);parser.add_argument('--capture-config',type=Path)
    parser.add_argument('--provenance-output',type=Path,help='Required for optional hardware capture; ignored private output')
    parser.add_argument('--batches',type=int,default=600)
    parser.add_argument('--geometry',choices=GEOMETRY_MODES,default='prepared',
                        help='prepared: robot builds surfaces; unprepared: Quest builds them; points: legacy five-camera grayscale')
    parser.add_argument('--retain-seconds',type=float,help='0 disables retained history; defaults to 30 for RGB/six-camera and 0 for legacy five-camera')
    parser.add_argument('--gimbal-control-config',type=Path,help='Separate private control token/certificate; only for gimbal-demo')
    parser.add_argument('--gimbal-port',type=int,help='Simulation control listen port (default 7444); only for gimbal-demo')
    args=parser.parse_args(argv);integer(args.batches,1,MAX_BATCHES,'stream batches')
    integer(args.port,0,65535,'scene port')
    if args.retain_seconds is not None and (not math.isfinite(args.retain_seconds) or not 0<=args.retain_seconds<=60):raise ValueError('retention must be finite seconds in [0,60]')
    if args.source!='gimbal-demo' and (args.gimbal_control_config is not None or args.gimbal_port is not None):raise ValueError('gimbal control is available only for the simulated gimbal-demo source')
    if args.source=='gimbal-demo':
        if args.geometry=='points':raise ValueError('gimbal RGB demo requires prepared or unprepared geometry')
        if args.gimbal_port is None:args.gimbal_port=7444
        integer(args.gimbal_port,0,65535,'gimbal port')
        if args.gimbal_control_config and args.port and args.gimbal_port==args.port:raise ValueError('scene and control ports must differ')
    if args.geometry=='points' and args.retain_seconds:raise ValueError('retained scenes require prepared or unprepared geometry')
    return args


def retention_policy(args,rgb):
    seconds=(30. if rgb else 0.) if args.retain_seconds is None else args.retain_seconds
    return seconds>0,max(1,int(math.ceil(seconds*1000))) if seconds>0 else 30000


def _server_config(path):
    config=load_json(path,16384);fields(config,('certificate','private_key','token_hex'))
    token=bytes.fromhex(config['token_hex'])
    if len(token)!=32:raise ValueError('server token must contain 32 bytes')
    return config,token


def main(argv=None):
    args=parse_args(argv);server_config,scene_token=_server_config(args.server_config)
    with ExitStack() as stack:
        capture=replay=demo=None;use_rgb=args.source=='gimbal-demo';recording=None
        if args.source=='replay':
            if args.manifest is None:raise ValueError('replay manifest required')
            recording=Recording(args.manifest);use_rgb=recording.data['version']==2
        elif args.source=='realsense':
            if args.capture_config is None or args.provenance_output is None:raise ValueError('private capture config and provenance output required')
            from .realsense import Capture
            if args.provenance_output.exists():raise ValueError('provenance output already exists')
            args.provenance_output.parent.mkdir(parents=True,exist_ok=True)
            capture=stack.enter_context(Capture(load_json(args.capture_config)))
            use_rgb=any(c.camera_id>4 or c.image_format!='Y8' for c in capture.cameras) or getattr(capture,'config',{}).get('version',1)==2
            args.provenance_output.write_text(json.dumps(dict(version=1,sources=capture.sources,
                clock='D400 global readout + exposure correction -> fixed host monotonic offset',
                host_wall_minus_monotonic_ns=capture.clock.offset),allow_nan=False,indent=2),encoding='utf-8')
        retained,retention_ms=retention_policy(args,use_rgb);use_rgb=use_rgb or retained
        if use_rgb and args.geometry=='points':raise ValueError('RGB/six-camera/retained scenes require prepared or unprepared geometry')
        from .observation_map import RetainedObservationMap
        cache=RetainedObservationMap(retention_ms=retention_ms) if use_rgb else None
        if recording is not None:
            replay=iter(recording.packets(geometry=args.geometry,rgb=use_rgb,observation_map=cache,retention_enabled=retained,retention_ms=retention_ms))
        control=None
        if args.source=='gimbal-demo':
            from .gimbal_demo import GimbalDemoSource
            demo=GimbalDemoSource(retention_ms=retention_ms);cache=demo.observation_map
            if args.gimbal_control_config is not None:
                from .gimbal_server import SimulatedGimbalServer
                config,token=_server_config(args.gimbal_control_config)
                if hmac.compare_digest(token,scene_token):raise ValueError('scene and gimbal control tokens must differ')
                control=stack.enter_context(SimulatedGimbalServer(demo.driver,config['certificate'],config['private_key'],token,args.host,args.gimbal_port,
                    reset_map=demo.request_clear,map_generation=lambda:demo.map_generation,source_id=demo.source_id))
                demo.server=control
        publisher=stack.enter_context(LatestPublisher(server_config['certificate'],server_config['private_key'],scene_token,args.host,args.port))
        epoch=secrets.randbits(64) or 1
        print(json.dumps(dict(listening=True,port=publisher.address[1],source=args.source,geometry=args.geometry,
            retention_enabled=retained,retention_ms=retention_ms,simulated_control_port=control.address[1] if control else None,requested_update_hz=10)),flush=True)
        count=overruns=0
        try:
            for sequence in range(1,args.batches+1):
                started=time.monotonic();tracking=True
                if replay is not None:
                    try:packet,_=next(replay)
                    except StopIteration:break
                else:
                    if demo is not None:
                        cameras,frames,tracking=demo.sample(sequence)
                        options=dict(source_id=demo.source_id,world_epoch=epoch,calibration_id=1,rig_mode='posed',observer_pose=OBSERVER,synthetic=True)
                    elif capture is None:
                        cameras,frames=synthetic_batch(capture_ns=time.monotonic_ns());stamp=min(f.capture_ns for f in frames)
                        frames=[replace(frame,sequence=sequence,capture_ns=stamp) for frame in frames]
                        options=dict(source_id=0x5253434e44454d4f,world_epoch=epoch,calibration_id=1,rig_mode='stationary',observer_pose=OBSERVER,synthetic=True)
                    else:
                        cameras=capture.cameras;frames=capture.poll();config=capture.config;tracking=getattr(capture,'tracking_valid',True)
                        options={key:config[key] for key in ('source_id','world_epoch','calibration_id','rig_mode','observer_pose')}
                    packet,_=build_batch(frames,cameras,geometry=args.geometry,rgb=use_rgb,sequence=sequence,produced_ns=time.monotonic_ns(),
                        observation_map=cache,tracking_valid=tracking,retention_enabled=retained,retention_ms=retention_ms,**options)
                    # V3 retention/masks were validated against this exact producer time.
                    # Late restamping could expire retained observations during packing.
                    if getattr(packet,'protocol_version',0)!=3:packet=replace(packet,produced_ns=time.monotonic_ns())
                publisher.publish(pack_packet(packet));count+=1
                if demo is not None:demo.observe_packet(packet)
                remaining=.1-(time.monotonic()-started)
                if remaining<0:overruns+=1
                time.sleep(max(0.,remaining))
        except KeyboardInterrupt:pass
        print(json.dumps(dict(published=count,overruns_100ms=overruns,sent_frames=publisher.sent_frames,
                              qualification='host process only; no camera/Quest throughput claim')),flush=True)

if __name__=='__main__':main()
