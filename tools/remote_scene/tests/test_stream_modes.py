"""Launch-selected representations keep one publisher and raw replay source."""
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch


class StreamModeTests(unittest.TestCase):
    def test_launch_modes_and_default(self):
        from tools.remote_scene.stream import parse_args
        base=['--source','demo','--server-config','unused.json']
        self.assertEqual(parse_args(base).geometry, 'prepared')
        for mode in ('points','prepared','unprepared'):
            self.assertEqual(parse_args(base+['--geometry',mode]).geometry,mode)
        with self.assertRaises(SystemExit):
            parse_args(base+['--geometry','unknown'])

    def test_dispatch_preserves_point_path_and_selects_surface_location(self):
        from tools.remote_scene.modes import build_batch
        with patch('tools.remote_scene.modes.fuse_batch', return_value=('old','stats')) as old:
            self.assertEqual(build_batch([],[],geometry='points',produced_ns=7),('old','stats'))
            old.assert_called_once_with([],[],produced_ns=7)
        for mode in ('prepared','unprepared'):
            with patch('tools.remote_scene.surface.build_surface_batch', return_value=('surface','stats')) as new:
                self.assertEqual(build_batch([],[],geometry=mode,produced_ns=8,voxel_size_m=.02,max_points=12),('surface','stats'))
                new.assert_called_once_with([],[],geometry=mode,produced_ns=8)
        with self.assertRaises(ValueError):
            build_batch([],[],geometry='unknown')

    def test_raw_recording_replays_all_modes_with_original_clock(self):
        from tools.remote_scene.demo import synthetic_batch,OBSERVER
        from tools.remote_scene.recording import write_recording,Recording
        from tools.remote_scene.protocol import pack_packet,unpack_packet,RECORDED
        from tools.remote_scene.surface import prepare_surface
        import numpy as np
        cameras,frames=synthetic_batch(24,18)
        with tempfile.TemporaryDirectory() as directory:
            manifest=write_recording(Path(directory),cameras,[(1005000000,frames)],
                source_id=17,world_epoch=2,calibration_id=1,rig_mode='stationary',observer_pose=OBSERVER,synthetic=True)
            packets={mode:next(Recording(manifest).packets(geometry=mode))[0]
                     for mode in ('points','prepared','unprepared')}
            for mode,packet in packets.items():
                with self.subTest(mode=mode):
                    decoded=unpack_packet(pack_packet(packet))
                    self.assertEqual(decoded.produced_ns,1005000000)
                    self.assertTrue(decoded.flags & RECORDED)
                    self.assertEqual(decoded.identity,packet.identity)
            built=prepare_surface(packets['unprepared'])
            np.testing.assert_array_equal(packets['prepared'].vertices,built.vertices)
            np.testing.assert_array_equal(packets['prepared'].indices,built.indices)
            np.testing.assert_array_equal(packets['prepared'].atlas,built.atlas)


if __name__=='__main__':unittest.main()
