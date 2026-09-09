"""Optional, hardware-unqualified USB D400 Z16/Y8 and D435i RGB8 capture.

Importing this module does not import the SDK or enumerate/connect cameras.
Only explicit Capture.open() / the capture CLI starts configured cameras.
Other product families or distortion/clock contracts require a separate audit.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import time
import numpy as np
from .calibration import CameraCalibration,DepthFrame,Intrinsics,rigid
from .recording import fields,load_json,write_recording,calibration_record,MAX_BATCHES
from .protocol import integer
from .pose_history import PoseUnavailable,read_pose_trace
from .gimbal import RigPoseProvider

SDK_AUDIT_COMMIT='e15c5d6bb1563e778d116f682aeefffbae2daedc'


def _native_intrinsics(value):
    return dict(width=int(value.width),height=int(value.height),fx=float(value.fx),fy=float(value.fy),
                ppx=float(value.ppx),ppy=float(value.ppy),model=str(value.model),coeffs=list(map(float,value.coeffs)))


class RgbRectifier:
    """Cache an SDK-forward inverse lookup; no handwritten distortion formula.

    Output pixels are undistorted pinhole rays projected into native RGB with
    rs2_project_point_to_pixel. Nearest sampling is explicit. Crop to an entirely
    valid rectangle, adjusting principal point, so an invented black border is
    never mislabeled as observed color. Verify profile and SDK probes on use.
    Supported models are the audited forward none/Brown/modified-Brown paths.
    """
    def __init__(self,rs,native):
        self.rs=rs;self.native=rs.intrinsics()
        record=_native_intrinsics(native)
        integer(record['width'],2,1920,'RGB width');integer(record['height'],2,1080,'RGB height')
        if any(not math.isfinite(record[name]) for name in ('fx','fy','ppx','ppy')) or min(record['fx'],record['fy'])<=0:
            raise ValueError('invalid native RGB intrinsics')
        if len(record['coeffs'])!=5 or not all(math.isfinite(v) for v in record['coeffs']):raise ValueError('invalid RGB distortion coefficients')
        supported=[getattr(rs.distortion,name,None) for name in ('none','brown_conrady','modified_brown_conrady')]
        if native.model not in [value for value in supported if value is not None]:raise ValueError('RGB distortion model requires another projection audit')
        if native.model==rs.distortion.none and any(record['coeffs']):raise ValueError('distortion-none RGB has nonzero coefficients')
        for name in ('width','height','fx','fy','ppx','ppy'):setattr(self.native,name,getattr(native,name))
        self.native.coeffs=list(native.coeffs)
        self.native.model=native.model;self.record=record
        width,height=record['width'],record['height']
        lookup=np.full((height,width),-1,dtype=np.int32)
        for y in range(height):
            for x in range(width):
                uv=self._project(x,y)
                if all(math.isfinite(v) for v in uv) and -.5<=uv[0]<width-.5 and -.5<=uv[1]<height-.5:
                    ix,iy=map(lambda v:int(math.floor(v+.5)),uv);lookup[y,x]=iy*width+ix
        invalid=(lookup<0).astype(np.int32)
        sums=np.pad(invalid,((1,0),(1,0))).cumsum(axis=0,dtype=np.int32).cumsum(axis=1,dtype=np.int32)
        def total(x0,y0,x1,y1):return int(sums[y1,x1]-sums[y0,x1]-sums[y1,x0]+sums[y0,x0])
        x0=y0=0;x1=width;y1=height
        # At most width+height edge removals; do not repeatedly scan whole images.
        while total(x0,y0,x1,y1):
            choices=[]
            if y1-y0>2:
                choices.extend(((total(x0,y0,x1,y0+1)/(x1-x0),'top'),(total(x0,y1-1,x1,y1)/(x1-x0),'bottom')))
            if x1-x0>2:
                choices.extend(((total(x0,y0,x0+1,y1)/(y1-y0),'left'),(total(x1-1,y0,x1,y1)/(y1-y0),'right')))
            if not choices or max(value for value,_ in choices)==0:raise ValueError('RGB lookup lacks a usable all-valid rectangle')
            _,edge=max(choices,key=lambda item:item[0])
            if edge=='top':y0+=1
            elif edge=='bottom':y1-=1
            elif edge=='left':x0+=1
            else:x1-=1
        self.roi=(x0,y0,x1,y1);self.lookup=lookup[y0:y1,x0:x1].copy();self.lookup.setflags(write=False)
        self.intrinsics=Intrinsics(x1-x0,y1-y0,record['fx'],record['fy'],record['ppx']-x0,record['ppy']-y0)
        self.intrinsics.validate()
        self.probes=[(x,y,tuple(self._project(x,y))) for y in sorted({y0,(y0+y1-1)//2,y1-1})
                     for x in sorted({x0,(x0+x1-1)//2,x1-1})]
        self.provenance=dict(method='cached_sdk_forward_nearest_crop',native=record,rectified=asdict(self.intrinsics),
                            crop=list(self.roi),lookup_sha256=hashlib.sha256(self.lookup.tobytes()).hexdigest(),sdk_audit_commit=SDK_AUDIT_COMMIT)

    def _project(self,x,y):
        p=self.native
        result=tuple(map(float,self.rs.rs2_project_point_to_pixel(p,[(x-p.ppx)/p.fx,(y-p.ppy)/p.fy,1.])))
        if len(result)!=2:raise ValueError('SDK RGB projection must return two coordinates')
        return result

    def apply(self,image,native):
        if _native_intrinsics(native)!=self.record:raise ValueError('RGB intrinsics changed; new calibration required')
        for x,y,expected in self.probes:
            actual=self._project(x,y)
            if len(actual)!=2 or not all(math.isfinite(v) for v in actual) or not np.allclose(actual,expected,rtol=0,atol=1e-5):
                raise ValueError('SDK RGB projection no longer matches cached lookup')
        value=np.asarray(image)
        if value.dtype!=np.dtype('u1') or value.shape!=(self.record['height'],self.record['width'],3):raise ValueError('RGB bytes do not match started profile')
        return value.reshape(-1,3)[self.lookup].copy()


def rgb_motion_gate(camera,depth_ns,image_ns,provider,policy):
    """Conservative static-scene gate, never rolling-shutter correction.

    support_window_ms is an operator-qualified radius about the reported image
    exposure time covering exposure and row readout. None is unqualified.
    """
    window=policy['support_window_ms']
    if window is None:return False,'RGB exposure/readout support window is unqualified'
    if provider is None or not callable(getattr(provider,'motion_bounds',None)):
        return False,'measured RGB motion-window coverage is unavailable'
    radius=int(math.ceil(window*1_000_000))
    start=min(depth_ns,image_ns)-radius;end=max(depth_ns,image_ns)+radius
    if start<=0:return False,'RGB support window precedes producer clock'
    # Different optical centers can have different translation under rotation.
    # Bound both sensors, including the depth-to-color calibration lever arm.
    image_sensor=rigid(camera.rig_from_depth) @ np.linalg.inv(rigid(camera.intensity_from_depth))
    try:
        bounds=[provider.motion_bounds(camera.camera_id,start,end,sensor)
                for sensor in (camera.rig_from_depth,image_sensor)]
    except (PoseUnavailable,NotImplementedError):return False,'RGB support window is not covered by tracked measured poses'
    for bound in bounds:
        translation=getattr(bound,'translation_m',None);rotation=getattr(bound,'rotation_rad',None)
        if (not isinstance(translation,(int,float)) or not isinstance(rotation,(int,float))
                or not math.isfinite(translation) or not math.isfinite(rotation)
                or translation<0 or rotation<0):return False,'invalid measured RGB motion bounds'
        if translation>policy['max_translation_m'] or rotation>policy['max_rotation_rad']:
            return False,'RGB motion exceeds qualified support-window limits'
    return True,'RGB measured motion-window gate passed'


def pose_provider_from_config(config):
    """Optional read-only trace adapter; no localization or encoder driver."""
    value=config.get('pose')
    if value is None:return None
    trace=read_pose_trace(value['trace_file'])
    return RigPoseProvider(trace.robot,trace.gimbal,robot_from_mount=value['robot_from_mount'],
        pan_from_tilt=value['pan_from_tilt'],gimbal_camera_id=value['gimbal_camera_id'])


def extrinsic_matrix(extrinsic):
    matrix=np.eye(4)
    # rs2_extrinsics stores rotation in column-major order.
    matrix[:3,:3]=np.asarray(extrinsic.rotation,dtype=float).reshape(3,3,order='F')
    matrix[:3,3]=extrinsic.translation
    return rigid(matrix,'queried intensity_from_depth')


def queried_intrinsics(value,none_model):
    if value.model!=none_model:raise ValueError('capture requires a queried rectified distortion-none profile')
    result=Intrinsics(int(value.width),int(value.height),float(value.fx),float(value.fy),
                      float(value.ppx),float(value.ppy),'none',tuple(float(x) for x in value.coeffs))
    result.validate();return result


def queried_usb_transport(device,camera_info):
    """Fail closed unless SDK information proves the audited USB path.

    Product line alone is insufficient: D400 MIPI firmware can expose an
    optical_timestamp offset as SENSOR_TIMESTAMP instead of an absolute sensor
    timestamp. Connection-type information is required even on older SDKs.
    """
    connection=getattr(camera_info,'connection_type',None)
    descriptor=getattr(camera_info,'usb_type_descriptor',None)
    if connection is None or descriptor is None or not device.supports(connection) or not device.supports(descriptor):
        raise ValueError('queried USB connection type and descriptor required for exposure-clock audit')
    kind=device.get_info(connection);usb=device.get_info(descriptor)
    if kind!='USB' or usb not in ('2.0','3.0','3.1','3.2'):
        raise ValueError('capture exposure-clock audit covers recognized USB transport only')
    return dict(connection_type=kind,usb_type_descriptor=usb)


class ExposureClock:
    """USB D400 GLOBAL_TIME + sensor/readout metadata -> one host monotonic clock.

    This does not assert hardware synchronization or clock-fit accuracy. Reject
    system/hardware timestamp fallback and discontinuities, never relabel arrival
    time as exposure time. The D400 readout timestamp correction is deliberately
    limited to that queried product line and USB transport
    (ds_timestamp_reader_from_metadata, d400_device::register_metadata).
    """
    def __init__(self,*,wall_ns=None,monotonic_ns=None):
        if wall_ns is None:
            before=time.monotonic_ns();wall_ns=time.time_ns();after=time.monotonic_ns()
            if after-before>1000000:raise ValueError('host clock sampling uncertainty exceeds 1 ms')
            monotonic_ns=(before+after)//2
        if monotonic_ns is None:raise ValueError('both explicit clock samples required')
        self.offset=wall_ns-monotonic_ns

    def check_wall(self,wall_ns,monotonic_ns):
        if abs(wall_ns-monotonic_ns-self.offset)>2000000:raise ValueError('host wall clock changed; restart with a new source epoch')

    def convert(self,global_ms,frame_us,sensor_us,*,now_ns,max_age_ns=2000000000):
        if not math.isfinite(global_ms) or global_ms<=0:raise ValueError('invalid global frame timestamp')
        if type(frame_us) is not int or type(sensor_us) is not int or min(frame_us,sensor_us)<0:raise ValueError('missing exposure metadata')
        correction=sensor_us-frame_us
        if not -1000000<=correction<=1000000:raise ValueError('exposure/readout clock wrap or discontinuity')
        stamp=int(round(global_ms*1000000))+correction*1000-self.offset
        if not 0<stamp<=now_ns or (max_age_ns is not None and now_ns-stamp>max_age_ns):raise ValueError('exposure clock not current in host time')
        return stamp


def _validate_config(config):
    if not isinstance(config,dict):raise ValueError('capture config must be an object')
    version=integer(config.get('version'),1,2,'capture config version')
    names=('version','sdk_version','source_id','world_epoch','calibration_id','rig_mode','observer_pose','max_pair_skew_ms','cameras')
    fields(config,names+(('pose',) if version==2 else ()))
    for key in ('source_id','world_epoch','calibration_id'):integer(config[key],1,2**64-1,key)
    if not isinstance(config['sdk_version'],str) or not config['sdk_version'] or len(config['sdk_version'])>80:raise ValueError('explicit installed SDK version required')
    if config['rig_mode'] not in ('stationary','posed'):raise ValueError('explicit rig mode required')
    integer(config['max_pair_skew_ms'],1,5,'depth/intensity exposure skew')
    if not isinstance(config['cameras'],list) or not (len(config['cameras'])==5 if version==1 else 1<=len(config['cameras'])<=6):
        raise ValueError('v1 requires five cameras; v2 requires one to six')
    if version==2 and config['pose'] is not None:
        pose=config['pose'];fields(pose,('trace_file','gimbal_camera_id','robot_from_mount','pan_from_tilt'))
        if not isinstance(pose['trace_file'],str) or not Path(pose['trace_file']).is_absolute():raise ValueError('pose trace path must be absolute')
        integer(pose['gimbal_camera_id'],0,5,'gimbal camera ID');rigid(pose['robot_from_mount']);rigid(pose['pan_from_tilt'])
    ids=set();serials=set()
    for camera in config['cameras']:
        fields(camera,('camera_id','serial','depth','intensity','rig_from_depth')+(('image_format','rgb_policy') if version==2 else ()))
        camera_id=integer(camera['camera_id'],0,4 if version==1 else 5,'camera ID')
        if version==2:
            image_format=camera['image_format']
            if image_format not in ('Y8','RGB8'):raise ValueError('capture image format must be Y8 or RGB8')
            if image_format=='Y8' and camera['rgb_policy'] is not None:raise ValueError('Y8 does not use an RGB policy')
            if image_format=='RGB8':
                policy=camera['rgb_policy'];fields(policy,('support_window_ms','max_translation_m','max_rotation_rad'))
                for key,high in (('support_window_ms',500.),('max_translation_m',.1),('max_rotation_rad',math.pi)):
                    value=policy[key]
                    if key=='support_window_ms' and value is None:continue
                    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=high:
                        raise ValueError('invalid RGB support-window policy')
        serial=camera['serial']
        if not isinstance(serial,str) or not serial or len(serial)>128:raise ValueError('private camera serial required')
        if camera_id in ids or serial in serials:raise ValueError('duplicate configured camera')
        ids.add(camera_id);serials.add(serial);rigid(camera['rig_from_depth'])
        for key in ('depth','intensity'):
            profile=camera[key];fields(profile,('width','height','fps','index'))
            integer(profile['width'],1,1920,'profile width');integer(profile['height'],1,1080,'profile height')
            integer(profile['fps'],1,90,'profile FPS');integer(profile['index'],0,2,'stream index')
    # Reuse the wire observer validation before starting any camera.
    from .protocol import Packet,POINT_DTYPE,validate
    validate(Packet(config['source_id'],config['world_epoch'],1,config['calibration_id'],0,0,1,31,0,2,.01,20,
                    config['observer_pose'],np.empty(0,dtype=POINT_DTYPE)))


class Capture:
    """One current SDK frameset per logical camera; optional external pose provider.

    pose_provider(camera_id, exposure_ns) must return map_from_rig at that
    exposure. An explicit v2 pose trace supplies the bounded measured-pose
    adapter; no localization, encoder driver, extrapolation or control is added.
    """
    def __init__(self,config,pose_provider=None):
        config=deepcopy(config)
        _validate_config(config)
        if pose_provider is not None and config.get('pose') is not None:raise ValueError('choose either external pose provider or pose trace')
        if pose_provider is None:pose_provider=pose_provider_from_config(config)
        if config['rig_mode']=='posed' and pose_provider is None:raise ValueError('posed capture requires an external exposure-time pose provider')
        self.config=config;self.pose_provider=pose_provider;self.pipelines=[];self.cameras=[];self.profiles=[]
        self.rs=None;self.clock=None;self.previous={};self.last_metadata=[];self.sources=[]
        self.started_ns=0;self.clock_not_ready=0
        self.rectifiers={};self.last_images={};self.tracking_valid=True

    def open(self):
        if self.rs is not None:raise RuntimeError('capture cannot restart')
        actual_version=importlib.metadata.version('pyrealsense2')
        if actual_version!=self.config['sdk_version']:raise ValueError('installed SDK version differs from operator config')
        import pyrealsense2 as rs  # Optional dependency: deliberately lazy.
        self.rs=rs;self.clock=ExposureClock()
        try:
            for requested in sorted(self.config['cameras'],key=lambda c:c['camera_id']):
                pipeline=rs.pipeline();cfg=rs.config();cfg.enable_device(requested['serial'])
                rgb=requested.get('image_format','Y8')=='RGB8'
                image_stream=rs.stream.color if rgb else rs.stream.infrared
                image_format=rs.format.rgb8 if rgb else rs.format.y8
                for key,stream,fmt in (('depth',rs.stream.depth,rs.format.z16),('intensity',image_stream,image_format)):
                    p=requested[key];cfg.enable_stream(stream,p['index'],p['width'],p['height'],fmt,p['fps'])
                resolved=cfg.resolve(rs.pipeline_wrapper(pipeline));device=resolved.get_device()
                if device.get_info(rs.camera_info.product_line)!='D400':raise ValueError('capture timestamp audit currently covers D400 only')
                transport=queried_usb_transport(device,rs.camera_info)
                if rgb and (not device.supports(rs.camera_info.product_id) or device.get_info(rs.camera_info.product_id).upper()!='0B3A'):
                    raise ValueError('RGB exposure audit currently covers USB D435i only')
                sensor=device.first_depth_sensor()
                if not sensor.supports(rs.option.global_time_enabled):raise ValueError('global camera clock conversion unavailable')
                sensor.set_option(rs.option.global_time_enabled,1.)
                if rgb:
                    color_sensor=device.first_color_sensor()
                    if not color_sensor.supports(rs.option.global_time_enabled):raise ValueError('RGB global clock unavailable')
                    color_sensor.set_option(rs.option.global_time_enabled,1.)
                active=pipeline.start(cfg);self.pipelines.append(pipeline)
                dp=active.get_stream(rs.stream.depth,requested['depth']['index']).as_video_stream_profile()
                ip=active.get_stream(image_stream,requested['intensity']['index']).as_video_stream_profile()
                for key,profile,fmt in (('depth',dp,rs.format.z16),('intensity',ip,image_format)):
                    want=requested[key]
                    if (profile.width(),profile.height(),profile.fps(),profile.stream_index(),profile.format())!=(want['width'],want['height'],want['fps'],want['index'],fmt):
                        raise ValueError('SDK started a different stream profile')
                rectifier=RgbRectifier(rs,ip.get_intrinsics()) if rgb else None
                if rectifier is not None:self.rectifiers[requested['camera_id']]=rectifier
                camera=CameraCalibration(requested['camera_id'],queried_intrinsics(dp.get_intrinsics(),rs.distortion.none),
                    rectifier.intrinsics if rgb else queried_intrinsics(ip.get_intrinsics(),rs.distortion.none),float(sensor.get_depth_scale()),
                    requested['rig_from_depth'],extrinsic_matrix(dp.get_extrinsics_to(ip)),'RGB8' if rgb else 'Y8')
                self.cameras.append(camera);self.profiles.append((dp,ip))
                options={}
                for name in ('global_time_enabled','inter_cam_sync_mode','emitter_enabled','laser_power','enable_auto_exposure','exposure'):
                    option=getattr(rs.option,name,None)
                    if option is not None and sensor.supports(option):options[name]=float(sensor.get_option(option))
                self.sources.append(dict(camera_id=camera.camera_id,sdk_version=actual_version,
                    product_line='D400',transport=transport,firmware=device.get_info(rs.camera_info.firmware_version),
                    requested_profiles={key:requested[key] for key in ('depth','intensity')},
                    queried_calibration=dict(calibration_record(camera),image_format=camera.image_format),queried_options=options,
                    rgb_rectification=rectifier.provenance if rectifier else None,
                    rgb_global_time_enabled=float(color_sensor.get_option(rs.option.global_time_enabled)) if rgb else None,
                    rgb_policy=requested.get('rgb_policy'),hardware_qualified=False))
        except Exception:
            self.close()
            # SDK exceptions can contain serials/endpoints. Keep them out of logs.
            raise RuntimeError('configured camera startup/profile/clock validation failed; review private configuration') from None
        self.started_ns=time.monotonic_ns()
        return self

    def _stamp(self,frame,now,*,allow_stale=False):
        rs=self.rs
        if frame.get_frame_timestamp_domain()!=rs.timestamp_domain.global_time:raise ValueError('camera has not established a global timestamp domain')
        keys=(rs.frame_metadata_value.frame_timestamp,rs.frame_metadata_value.sensor_timestamp)
        if not all(frame.supports_frame_metadata(key) for key in keys):raise ValueError('exposure metadata unavailable')
        readout,exposure=(int(frame.get_frame_metadata(key)) for key in keys)
        global_ms=float(frame.get_timestamp())
        stamp=self.clock.convert(global_ms,readout,exposure,now_ns=now,max_age_ns=None if allow_stale else 2000000000)
        return stamp,dict(domain='global_time',frame_global_ms=global_ms,frame_timestamp_us=readout,sensor_timestamp_us=exposure)

    def _image(self,latest,camera,profile,request,current):
        rgb=camera.image_format=='RGB8'
        image=latest.get_color_frame() if rgb else latest.get_infrared_frame(request['intensity']['index'])
        cached=self.last_images.get(camera.camera_id) if rgb else None
        if not image:
            return None if cached is None else dict(cached,candidate=False,source='cached_missing_RGB')
        if image.profile.unique_id()!=profile.unique_id():raise ValueError('image profile changed; new calibration required')
        try:stamp,clock=self._stamp(image,current,allow_stale=rgb)
        except ValueError:
            if cached is not None:return dict(cached,candidate=False,source='cached_invalid_RGB_clock')
            if camera.camera_id not in self.previous and current-self.started_ns<2000000000:
                self.clock_not_ready+=1;return None
            raise
        sequence=int(image.get_frame_number())
        if cached is not None:
            if sequence==cached['sequence'] and stamp==cached['stamp']:
                return dict(cached,candidate=True,source='repeated_RGB_exposure')
            if sequence<=cached['sequence'] or stamp<=cached['stamp']:
                return dict(cached,candidate=False,source='cached_reversed_RGB_clock_or_sequence')
        pixels=np.asarray(image.get_data())
        if rgb:pixels=self.rectifiers[camera.camera_id].apply(pixels,profile.get_intrinsics())
        else:pixels=pixels.copy()
        result=dict(array=pixels,stamp=stamp,sequence=sequence,clock=clock,candidate=True,source='new_image')
        if rgb:self.last_images[camera.camera_id]=result
        return result

    def poll(self):
        if self.rs is None or not self.pipelines:raise RuntimeError('capture not open')
        frames=[];metadata=[];self.tracking_valid=True
        try:
            now=time.monotonic_ns();self.clock.check_wall(time.time_ns(),now)
            for pipeline,camera,profiles,request in zip(self.pipelines,self.cameras,self.profiles,sorted(self.config['cameras'],key=lambda c:c['camera_id'])):
                latest=None
                for _ in range(8):  # Bound backlog draining, even under an unusual SDK queue.
                    sample=pipeline.poll_for_frames()
                    if not sample:break
                    latest=sample
                if latest is None:continue
                depth=latest.get_depth_frame()
                if not depth:continue
                if depth.profile.unique_id()!=profiles[0].unique_id():raise ValueError('depth profile changed; new calibration required')
                current=time.monotonic_ns()
                try:
                    depth_stamp,depth_meta=self._stamp(depth,current)
                except ValueError:
                    # The SDK global fit may not be established on its first
                    # frames. Publish missing cameras for at most two seconds.
                    if camera.camera_id not in self.previous and current-self.started_ns<2000000000:
                        self.clock_not_ready+=1;continue
                    raise
                image=self._image(latest,camera,profiles[1],request,current)
                if image is None:continue  # No timestamp is invented before the first valid image.
                gray_stamp,gray_number=image['stamp'],image['sequence']
                version=self.config['version'];number=int(depth.get_frame_number())
                previous=self.previous.get(camera.camera_id)
                if previous is not None:
                    if version==2 and number==previous[0] and depth_stamp==previous[1]:continue
                    if number<=previous[0] or depth_stamp<=previous[1] or (version==1 and (gray_number<=previous[2] or gray_stamp<=previous[3])):
                        raise ValueError('camera sequence or clock reversed/repeated')
                pair_ok=abs(depth_stamp-gray_stamp)<=self.config['max_pair_skew_ms']*1000000
                if version==1 and not pair_ok:continue
                image_usable=pair_ok and image['candidate']
                reason='image exposure pair accepted' if image_usable else 'image is missing, reversed or outside exposure-pair skew'
                try:pose=None if self.pose_provider is None else rigid(self.pose_provider(camera.camera_id,depth_stamp),'capture-time pose')
                except PoseUnavailable:
                    self.tracking_valid=False
                    metadata.append(dict(camera_id=camera.camera_id,capture_ns=depth_stamp,dropped='measured depth pose unavailable'))
                    continue
                image_from_depth=None
                if version==2 and self.pose_provider is not None:
                    try:
                        image_rig=rigid(self.pose_provider(camera.camera_id,gray_stamp),'image-exposure pose')
                        world_depth=pose@camera.rig_from_depth;image_time_depth=image_rig@camera.rig_from_depth
                        image_from_depth=rigid(camera.intensity_from_depth@np.linalg.inv(image_time_depth)@world_depth,'cross-exposure image pose')
                    except PoseUnavailable:image_usable=False;reason='measured image-exposure pose unavailable'
                if version==2 and camera.image_format=='RGB8' and image_usable:
                    image_usable,reason=rgb_motion_gate(camera,depth_stamp,gray_stamp,self.pose_provider,request['rgb_policy'])
                # SDK memory is recycled; the captured arrays own independent bytes.
                values={} if version==1 else dict(image_capture_ns=gray_stamp,image_from_depth=image_from_depth,image_usable=bool(image_usable))
                frame=DepthFrame(camera.camera_id,number,depth_stamp,np.asarray(depth.get_data()).copy(),image['array'].copy(),pose,**values)
                frame.validate(camera,self.config['rig_mode']);frames.append(frame)
                self.previous[camera.camera_id]=number,depth_stamp,gray_number,gray_stamp
                metadata.append(dict(camera_id=camera.camera_id,sequence=number,capture_ns=depth_stamp,
                    intensity_sequence=gray_number,intensity_capture_ns=gray_stamp,depth=depth_meta,intensity=image['clock'],
                    image_format=camera.image_format,image_source=image['source'],image_usable=bool(image_usable),image_reason=reason))
        except Exception:
            self.close()
            raise RuntimeError('camera frame/profile/exposure validation failed; capture stopped') from None
        self.last_metadata=metadata
        return frames

    def close(self):
        for pipeline in self.pipelines:
            try:pipeline.stop()
            except Exception:pass
        self.pipelines.clear()
        if hasattr(self,'last_images'):self.last_images.clear()
        if hasattr(self,'rectifiers'):self.rectifiers.clear()

    def __enter__(self):return self.open()
    def __exit__(self,*_):self.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path,help='Ignored private v1 five-camera or v2 RGB/pose config')
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--batches',type=int,default=100,help='10 Hz, at most 600 batches')
    args=parser.parse_args();integer(args.batches,1,MAX_BATCHES,'capture batches');config=load_json(args.config)
    if config.get('rig_mode')=='posed' and config.get('pose') is None:raise ValueError('posed CLI capture requires an explicit measured pose trace')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if any(args.output_dir.iterdir()):raise ValueError('capture output directory must be empty and ignored')
    provenance=[]
    with Capture(config) as capture:
        def batches():
            for _ in range(args.batches):
                started=time.monotonic();frames=capture.poll();produced=time.monotonic_ns()
                provenance.append(dict(produced_ns=produced,frames=capture.last_metadata))
                yield produced,frames,capture.tracking_valid
                time.sleep(max(0.,.1-(time.monotonic()-started)))
        write_recording(args.output_dir,capture.cameras,batches(),source_id=config['source_id'],world_epoch=config['world_epoch'],
            calibration_id=config['calibration_id'],rig_mode=config['rig_mode'],observer_pose=config['observer_pose'])
        (args.output_dir/'capture-provenance.json').write_text(json.dumps(dict(version=1,sources=capture.sources,
            clock='D400 global readout + exposure correction -> fixed host monotonic offset',
            host_wall_minus_monotonic_ns=capture.clock.offset,batches=provenance),allow_nan=False,indent=2),encoding='utf-8')
    print('Configured capture saved. Hardware synchronization and reconstruction accuracy require separate qualification.')


if __name__=='__main__':main()
