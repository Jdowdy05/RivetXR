import tempfile
import unittest
from dataclasses import replace
import numpy as np
from tools.remote_scene.tests.test_rgb import batch
from tools.remote_scene.recording import write_recording,Recording,load_json
from tools.remote_scene.rgb_surface import prepare_rgb
from tools.remote_scene.protocol import pack_packet

class RGBRecordingTests(unittest.TestCase):
    def test_rgb_raw_recording_preserves_exposures_pose_and_pixels(self):
        c,f=batch();pose=np.eye(4);pose[0,3]=.015
        f=[replace(f[0],image_capture_ns=999000000,image_from_depth=pose,image_usable=False)]
        with tempfile.TemporaryDirectory() as folder:
            path=write_recording(folder,c,[(1001000000,f)],source_id=1,world_epoch=1,calibration_id=1,
                                 rig_mode='stationary',observer_pose=(0,0,1,0,0,0,1))
            self.assertEqual(load_json(path)['version'],2)
            raw,_=next(Recording(path).packets(geometry='unprepared'))
            self.assertEqual(raw.views[0].image_capture_ns,999000000);self.assertFalse(raw.views[0].flags&2)
            np.testing.assert_array_equal(raw.views[0].intensity,f[0].intensity)
            prepared,_=next(Recording(path).packets(geometry='prepared'))
            np.testing.assert_array_equal(prepare_rgb(raw).vertices,prepared.vertices)
            with self.assertRaises(ValueError):next(Recording(path).packets(geometry='points'))

    def test_legacy_recording_schema_and_default_remain_v1_points(self):
        c,f=batch(rgb=False)
        with tempfile.TemporaryDirectory() as folder:
            path=write_recording(folder,c,[(1001000000,f)],source_id=1,world_epoch=1,calibration_id=1,
                                 rig_mode='stationary',observer_pose=(0,0,1,0,0,0,1))
            self.assertEqual(load_json(path)['version'],1)
            packet,_=next(Recording(path).packets());self.assertEqual(pack_packet(packet)[4:6],b'\x01\x00')

    def test_tracking_loss_roundtrips_and_retires_replay_history(self):
        c,f=batch();first=f[0]
        pose=np.eye(4);pose[0,3]=.2
        second=replace(first,sequence=2,capture_ns=1100000000,world_from_rig=pose)
        fourth=replace(second,sequence=3,capture_ns=1300000000)
        first=replace(first,world_from_rig=np.eye(4))
        with tempfile.TemporaryDirectory() as folder:
            path=write_recording(folder,c,[(1001000000,[first],True),(1101000000,[second],True),
                                  (1201000000,[],False),(1301000000,[fourth],True)],source_id=1,world_epoch=1,calibration_id=1,
                                  rig_mode='posed',observer_pose=(0,0,1,0,0,0,1))
            manifest=load_json(path);self.assertEqual(manifest['version'],2)
            self.assertFalse(manifest['batches'][2]['tracking_valid'])
            self.assertTrue(manifest['batches'][0]['frames'][0]['intensity_file'].endswith('.rgb8'))
            packets=[p for p,_ in Recording(path).packets(geometry='unprepared',retention_enabled=True)]
            self.assertTrue(any(v.flags&1 for v in packets[1].views))
            self.assertFalse(packets[2].views);self.assertFalse(packets[2].map_flags&1)
            self.assertGreater(packets[2].map_generation,packets[1].map_generation)
            self.assertEqual(packets[3].map_generation,packets[2].map_generation)
            self.assertFalse(any(v.flags&1 for v in packets[3].views))

if __name__=='__main__':unittest.main()
