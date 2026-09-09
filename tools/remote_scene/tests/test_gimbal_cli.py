import unittest
from tools.remote_scene.stream import parse_args

class GimbalCliTests(unittest.TestCase):
    def test_gimbal_source_and_retention_options_are_explicit(self):
        args=parse_args(['--source','gimbal-demo','--server-config','unused.json'])
        self.assertEqual(args.geometry,'prepared');self.assertIsNone(args.retain_seconds)
        args=parse_args(['--source','gimbal-demo','--server-config','unused.json','--retain-seconds','0'])
        self.assertEqual(args.retain_seconds,0)

    def test_invalid_retention_points_and_control_source_combinations_reject(self):
        for extra in (['--retain-seconds','nan'],['--retain-seconds','61'],['--geometry','points'],
                      ['--retain-seconds','-1']):
            with self.subTest(extra=extra),self.assertRaises(ValueError):
                parse_args(['--source','gimbal-demo','--server-config','unused.json']+extra)
        for source in ('demo','realsense','replay'):
            with self.subTest(source=source),self.assertRaises(ValueError):
                parse_args(['--source',source,'--server-config','unused.json','--gimbal-control-config','unused-control.json'])

    def test_capture_tracking_loss_publishes_empty_retired_map(self):
        import contextlib,io,json,tempfile,time
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import patch
        from tools.remote_scene.tests.test_rgb import batch
        from tools.remote_scene.protocol import unpack_packet
        from tools.remote_scene.stream import main
        cameras,frames=batch(stamp=time.monotonic_ns())
        captured=[]
        class Capture:
            def __init__(self,config):
                self.cameras=cameras;self.tracking_valid=False;self.clock=SimpleNamespace(offset=0);self.sources=[]
                self.config=dict(version=2,source_id=1,world_epoch=1,calibration_id=1,rig_mode='stationary',observer_pose=(0,0,1,0,0,0,1))
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def poll(self):return frames
        class Publisher:
            address=('127.0.0.1',1234);sent_frames=0
            def __init__(self,*args,**kwargs):pass
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def publish(self,raw):captured.append(unpack_packet(raw))
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);server=root/'server.json';config=root/'capture.json'
            server.write_text(json.dumps(dict(certificate='unused',private_key='unused',token_hex='00'*32)));config.write_text('{}')
            with patch('tools.remote_scene.realsense.Capture',Capture),patch('tools.remote_scene.stream.LatestPublisher',Publisher),contextlib.redirect_stdout(io.StringIO()):
                main(['--source','realsense','--server-config',str(server),'--capture-config',str(config),
                      '--provenance-output',str(root/'provenance.json'),'--batches','1'])
        self.assertEqual(len(captured),1);self.assertEqual(captured[0].contributing_mask,0)
        self.assertFalse(captured[0].map_flags&1);self.assertEqual(captured[0].map_generation,2)

    def test_shared_scene_control_token_rejected_before_network_start(self):
        import json,tempfile
        from pathlib import Path
        from tools.remote_scene.stream import main
        with tempfile.TemporaryDirectory() as folder:
            config=Path(folder)/'server.json';config.write_text(json.dumps(dict(certificate='unused',private_key='unused',token_hex='00'*32)))
            with self.assertRaisesRegex(ValueError,'tokens must differ'):
                main(['--source','gimbal-demo','--server-config',str(config),'--gimbal-control-config',str(config)])

if __name__=='__main__':unittest.main()
