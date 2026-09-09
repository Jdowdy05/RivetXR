"""Strict bounded manifest + exact raw Z16/Y8 files; deterministic offline replay."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re
import numpy as np
from .calibration import CameraCalibration,DepthFrame,Intrinsics,rigid
from .fusion import fuse_batch
from .protocol import integer,pack_packet

MAX_MANIFEST_BYTES=4*1024*1024
MAX_BATCHES=600


def unique(pairs):
    result={}
    for key,value in pairs:
        if key in result:raise ValueError('duplicate JSON field')
        result[key]=value
    return result


def load_json(path,limit=MAX_MANIFEST_BYTES):
    path=Path(path)
    if path.stat().st_size>limit:raise ValueError('JSON file exceeds bound')
    with path.open('rb') as stream:raw=stream.read(limit+1)
    if len(raw)>limit:raise ValueError('JSON file grew beyond bound')
    return json.loads(raw.decode('utf-8'),object_pairs_hook=unique,
                      parse_constant=lambda _:(_ for _ in ()).throw(ValueError('nonfinite JSON constant')))


def fields(value,names):
    if not isinstance(value,dict) or set(value)!=set(names):raise ValueError('unknown or missing manifest fields')


def _intrinsics(value):
    fields(value,('width','height','fx','fy','ppx','ppy','distortion','coeffs'))
    if not isinstance(value['coeffs'],list):raise ValueError('invalid distortion coefficients')
    result=Intrinsics(**dict(value,coeffs=tuple(value['coeffs'])));result.validate();return result


def calibration_record(camera,version=1):
    result=dict(camera_id=camera.camera_id,depth=asdict(camera.depth),intensity=asdict(camera.intensity),
                depth_units_m=camera.depth_units_m,rig_from_depth=camera.rig_from_depth.ravel().tolist(),
                intensity_from_depth=camera.intensity_from_depth.ravel().tolist())
    if version==2:result['image_format']=camera.image_format
    return result


def _camera(value,version=1):
    fields(value,('camera_id','depth','intensity','depth_units_m','rig_from_depth','intensity_from_depth')+ (('image_format',) if version==2 else ()))
    return CameraCalibration(value['camera_id'],_intrinsics(value['depth']),_intrinsics(value['intensity']),
                             value['depth_units_m'],value['rig_from_depth'],value['intensity_from_depth'],image_format=value.get('image_format','Y8'))


def _raw(root,name,checksum,size,dtype,shape):
    if not isinstance(name,str) or Path(name).is_absolute():raise ValueError('frame path must be relative')
    path=(root/name).resolve()
    if root not in path.parents:raise ValueError('frame path escapes recording')
    if not isinstance(checksum,str) or not re.fullmatch('[0-9a-f]{64}',checksum):raise ValueError('invalid frame checksum')
    if path.stat().st_size!=size:raise ValueError('frame byte count differs from calibration')
    with path.open('rb') as stream:raw=stream.read(size+1)
    if len(raw)!=size or hashlib.sha256(raw).hexdigest()!=checksum:raise ValueError('frame changed or checksum mismatch')
    return np.frombuffer(raw,dtype=dtype).reshape(shape)


class Recording:
    def __init__(self,manifest):
        manifest=Path(manifest);self.root=manifest.parent.resolve();self.data=load_json(manifest)
        d=self.data
        fields(d,('version','source_id','world_epoch','calibration_id','rig_mode','clock','observer_pose',
                  'voxel_size_m','max_capture_skew_ms','max_points','synthetic','cameras','batches'))
        integer(d['version'],1,2,'manifest version')
        for key in ('source_id','world_epoch','calibration_id'):integer(d[key],1,2**64-1,key)
        if d['rig_mode'] not in ('stationary','posed') or d['clock']!='producer_monotonic_ns' or type(d['synthetic']) is not bool:raise ValueError('invalid rig/clock/source declaration')
        if not isinstance(d['cameras'],list) or not 1<=len(d['cameras'])<=(6 if d['version']==2 else 5):raise ValueError('invalid camera count')
        self.cameras=[_camera(c,d['version']) for c in d['cameras']]
        if len({c.camera_id for c in self.cameras})!=len(self.cameras):raise ValueError('duplicate camera ID')
        if not isinstance(d['batches'],list) or not 1<=len(d['batches'])<=MAX_BATCHES:raise ValueError('invalid batch count')
        # Validate packet-level settings even if every depth sample is invalid.
        from dataclasses import replace
        settings_cameras=self.cameras if d['version']==1 else [replace(self.cameras[0],camera_id=0,image_format='Y8')]
        fuse_batch([],settings_cameras,source_id=d['source_id'],world_epoch=d['world_epoch'],sequence=1,
            calibration_id=d['calibration_id'],produced_ns=1,rig_mode=d['rig_mode'],voxel_size_m=d['voxel_size_m'],
            max_capture_skew_ms=d['max_capture_skew_ms'],max_points=d['max_points'],observer_pose=d['observer_pose'])
        # Reject invalid metadata before exposing any batch, without loading all
        # raw images into memory or computing the reconstruction twice.
        for _ in self._batches():pass

    def _batches(self):
        d=self.data;by_id={c.camera_id:c for c in self.cameras};previous={};produced_before=0
        for sequence,batch in enumerate(d['batches'],1):
            fields(batch,('produced_ns','frames')+(('tracking_valid',) if d['version']==2 and 'tracking_valid' in batch else ()))
            if type(batch.get('tracking_valid',True)) is not bool:raise ValueError('invalid recorded tracking state')
            produced=integer(batch['produced_ns'],1,2**64-1,'production time')
            if produced<=produced_before:raise ValueError('recorded batch time did not increase')
            produced_before=produced
            if not isinstance(batch['frames'],list) or len(batch['frames'])>(6 if d['version']==2 else 5):raise ValueError('invalid recorded frame count')
            seen=set()
            for frame in batch['frames']:
                fields(frame,('camera_id','sequence','capture_ns','depth_file','depth_sha256','intensity_file','intensity_sha256','world_from_rig')+ (('image_capture_ns','image_from_depth','image_usable') if d['version']==2 else ()))
                camera_id=integer(frame['camera_id'],0,5 if d['version']==2 else 4,'camera ID')
                if camera_id not in by_id or camera_id in seen:raise ValueError('unknown/duplicate batch camera')
                seen.add(camera_id);camera=by_id[camera_id]
                number=integer(frame['sequence'],1,2**64-1,'camera sequence');stamp=integer(frame['capture_ns'],1,2**64-1,'capture time')
                if camera_id in previous and (number<=previous[camera_id][0] or stamp<=previous[camera_id][1]):raise ValueError('camera frames repeated or reordered')
                previous[camera_id]=number,stamp
                if stamp>produced:raise ValueError('capture newer than production')
                if d['version']==2:
                    image_stamp=integer(frame['image_capture_ns'],1,produced,'image capture time')
                    if type(frame['image_usable']) is not bool:raise ValueError('invalid recorded image usability')
                    if frame['image_from_depth'] is not None:rigid(frame['image_from_depth'],'recorded exposure image_from_depth')
                pose=frame['world_from_rig']
                if d['rig_mode']=='posed' and pose is None:raise ValueError('capture-time world pose required')
                if pose is not None:
                    matrix=rigid(pose)
                    if d['rig_mode']=='stationary' and not np.array_equal(matrix,np.eye(4)):raise ValueError('stationary pose changed')
                for prefix in ('depth','intensity'):
                    name=frame[prefix+'_file'];checksum=frame[prefix+'_sha256']
                    if not isinstance(name,str) or Path(name).is_absolute() or self.root not in (self.root/name).resolve().parents:raise ValueError('frame path escapes recording')
                    if not isinstance(checksum,str) or not re.fullmatch('[0-9a-f]{64}',checksum):raise ValueError('invalid frame checksum')
            yield sequence,batch

    def packets(self, *, geometry='points',rgb=None,observation_map=None,retention_enabled=False,retention_ms=30000):
        from .modes import GEOMETRY_MODES,build_batch
        if geometry not in GEOMETRY_MODES:raise ValueError('invalid replay geometry mode')
        d=self.data;by_id={c.camera_id:c for c in self.cameras}
        use_rgb=d['version']==2 or retention_enabled if rgb is None else rgb
        if d['version']==2 and not use_rgb:raise ValueError('v2 recordings require RGB/per-exposure scene semantics')
        if use_rgb and geometry=='points':raise ValueError('RGB/six-camera recordings require prepared or unprepared geometry')
        if use_rgb:
            from .rgb_surface import build_rgb_batch
            from .observation_map import RetainedObservationMap
            if observation_map is None:observation_map=RetainedObservationMap(retention_ms=retention_ms)
        for sequence,batch in self._batches():
            frames=[];produced=batch['produced_ns']
            for frame in batch['frames']:
                camera_id=frame['camera_id'];camera=by_id[camera_id]
                depth=_raw(self.root,frame['depth_file'],frame['depth_sha256'],camera.depth.width*camera.depth.height*2,'<u2',(camera.depth.height,camera.depth.width))
                channels=3 if camera.image_format=='RGB8' else 1
                shape=(camera.intensity.height,camera.intensity.width)+((3,) if channels==3 else ())
                gray=_raw(self.root,frame['intensity_file'],frame['intensity_sha256'],camera.intensity.width*camera.intensity.height*channels,'u1',shape)
                frames.append(DepthFrame(camera_id,frame['sequence'],frame['capture_ns'],depth,gray,frame['world_from_rig'],frame.get('image_capture_ns'),frame.get('image_from_depth'),frame.get('image_usable',True)))
            builder=build_rgb_batch if use_rgb else build_batch
            options=dict(observation_map=observation_map,retention_enabled=retention_enabled,retention_ms=retention_ms,tracking_valid=batch.get('tracking_valid',True)) if use_rgb else dict(voxel_size_m=d['voxel_size_m'],max_points=d['max_points'])
            packet,stats=builder(frames,self.cameras,geometry=geometry,source_id=d['source_id'],world_epoch=d['world_epoch'],sequence=sequence,
                calibration_id=d['calibration_id'],produced_ns=produced,rig_mode=d['rig_mode'],
                max_capture_skew_ms=d['max_capture_skew_ms'],observer_pose=d['observer_pose'],recorded=True,synthetic=d['synthetic'],**options)
            yield packet,stats


def write_recording(directory,cameras,batches,*,source_id,world_epoch,calibration_id,rig_mode,observer_pose,synthetic=False,
                    voxel_size_m=.01,max_capture_skew_ms=20,max_points=50000):
    root=Path(directory);root.mkdir(parents=True,exist_ok=True)
    if (root/'manifest.json').exists():raise ValueError('recording already exists')
    records=[];version=2 if any(c.camera_id>4 or c.image_format!='Y8' for c in cameras) else 1
    for batch_index,batch in enumerate(batches):
        if not isinstance(batch,(tuple,list)) or len(batch) not in (2,3):raise ValueError('recorded batch requires production time, frames, optional tracking state')
        produced,frames=batch[:2];tracking=batch[2] if len(batch)==3 else True
        if type(tracking) is not bool:raise ValueError('recorded tracking state must be boolean')
        if not tracking:version=2
        if batch_index>=MAX_BATCHES:raise ValueError('recording exceeds batch limit')
        record=dict(produced_ns=produced,frames=[],tracking_valid=tracking)
        for frame in frames:
            camera=next((c for c in cameras if c.camera_id==frame.camera_id),None)
            if camera is None:raise ValueError('unknown frame camera')
            frame.validate(camera,rig_mode)
            if frame.image_capture_ns is not None or frame.image_from_depth is not None or not frame.image_usable:version=2
            item=dict(image_capture_ns=frame.image_timestamp_ns,image_from_depth=None if frame.image_from_depth is None else rigid(frame.image_from_depth).ravel().tolist(),
                      image_usable=frame.image_usable,camera_id=frame.camera_id,sequence=frame.sequence,capture_ns=frame.capture_ns,
                      world_from_rig=None if frame.world_from_rig is None else rigid(frame.world_from_rig).ravel().tolist())
            for prefix,array in (('depth',frame.depth),('intensity',frame.intensity)):
                name=f'b{batch_index:04d}_c{frame.camera_id}.{ "z16" if prefix=="depth" else "rgb8" if camera.image_format=="RGB8" else "y8"}'
                raw=array.tobytes(order='C');path=root/name
                with path.open('xb') as stream:stream.write(raw)
                item[prefix+'_file']=name;item[prefix+'_sha256']=hashlib.sha256(raw).hexdigest()
            record['frames'].append(item)
        records.append(record)
    if version==1:
        for record in records:
            record.pop('tracking_valid')
            for item in record['frames']:
                for field in ('image_capture_ns','image_from_depth','image_usable'):item.pop(field)
    data=dict(version=version,source_id=source_id,world_epoch=world_epoch,calibration_id=calibration_id,rig_mode=rig_mode,
              clock='producer_monotonic_ns',observer_pose=list(observer_pose),voxel_size_m=voxel_size_m,
              max_capture_skew_ms=max_capture_skew_ms,max_points=max_points,synthetic=synthetic,
              cameras=[calibration_record(c,version) for c in cameras],batches=records)
    raw=json.dumps(data,allow_nan=False,indent=2).encode()
    if len(raw)>MAX_MANIFEST_BYTES:raise ValueError('manifest exceeds bound')
    path=root/'manifest.json';pending=root/'manifest.pending';pending.write_bytes(raw)
    # Validate structure/calibration before exposing the complete manifest.
    Recording(pending);pending.replace(path);return path


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('manifest',type=Path);parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    count=0
    for packet,stats in Recording(args.manifest).packets():
        (args.output_dir/f'{packet.sequence:06d}.rscn').write_bytes(pack_packet(packet));count+=1
    print(json.dumps(dict(replayed_batches=count,scope='calibrated recorded current-view fusion; no live cameras')))


if __name__=='__main__':main()
