from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from tools.remote_scene.demo import synthetic_batch,OBSERVER
from tools.remote_scene.recording import Recording,write_recording,load_json
from tools.remote_scene.protocol import RECORDED,SYNTHETIC,pack_packet


class RecordingTests(unittest.TestCase):
    def make(self,root):
        cameras,frames=synthetic_batch(16,12)
        return write_recording(root,cameras,[(1005000000,frames)],source_id=1,world_epoch=1,
            calibration_id=1,rig_mode='stationary',observer_pose=OBSERVER,synthetic=True)

    def test_exact_raw_recording_and_replay(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest=self.make(temp)
            a=list(Recording(manifest).packets());b=list(Recording(manifest).packets())
            self.assertEqual(len(a),1);self.assertEqual(pack_packet(a[0][0]),pack_packet(b[0][0]))
            self.assertEqual(a[0][0].flags&(RECORDED|SYNTHETIC),RECORDED|SYNTHETIC)

    def test_raw_corruption_and_path_escape_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest=self.make(temp);data=load_json(manifest)
            frame=data['batches'][0]['frames'][0];raw=Path(temp)/frame['depth_file']
            content=bytearray(raw.read_bytes());content[0]^=1;raw.write_bytes(content)
            with self.assertRaises(ValueError):list(Recording(manifest).packets())
            frame['depth_file']='../outside.z16';manifest.write_text(json.dumps(data))
            with self.assertRaises(ValueError):Recording(manifest)

    def test_unknown_duplicate_fields_and_reordered_metadata_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest=self.make(temp);original=manifest.read_text();data=json.loads(original)
            manifest.write_text(original.replace('"version": 1','"version": 1, "version": 1'))
            with self.assertRaises(ValueError):Recording(manifest)
            data['batches'].append(data['batches'][0]);manifest.write_text(json.dumps(data))
            with self.assertRaises(ValueError):Recording(manifest)
            data=json.loads(original);data['batches'][0]['frames'][0]['world_from_rig']=np.diag([1,1,1,0]).ravel().tolist()
            manifest.write_text(json.dumps(data))
            with self.assertRaises(ValueError):Recording(manifest)

    def test_invalid_batch_never_exposes_manifest(self):
        with tempfile.TemporaryDirectory() as temp:
            cameras,frames=synthetic_batch(8,6)
            with self.assertRaises(ValueError):write_recording(temp,cameras,[(1,frames)],source_id=1,world_epoch=1,calibration_id=1,
                rig_mode='stationary',observer_pose=OBSERVER)
            self.assertFalse((Path(temp)/'manifest.json').exists())


if __name__=='__main__':unittest.main()
