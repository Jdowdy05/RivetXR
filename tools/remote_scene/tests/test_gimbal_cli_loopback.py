"""Real CLI subprocess + local pinned TLS; no camera, Quest or physical motor."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import numpy as np
from tools.remote_scene.provision import provision,DEFAULT_OPENSSL
from tools.remote_scene.publisher import connect_pinned,read_packet,recv_exact
from tools.remote_scene.protocol import unpack_packet
from tools.remote_scene.gimbal_control import Command,pack_command,unpack_feedback,HOLD,AIM,RESET_MAP,ARMED
from tools.remote_scene.gimbal_demo import SOURCE_ID,STATIC_SOURCE_ID

@unittest.skipUnless(DEFAULT_OPENSSL.is_file(),'test certificate helper required')
class GimbalCliLoopbackTests(unittest.TestCase):
    def test_scene_and_simulated_control_motion_reset_stale_and_disconnect(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);_,scene=provision(root,test_certificate=True,control_port=7444)
            control=json.loads((root/'remote-gimbal-client.json').read_text())
            command=[sys.executable,'-m','tools.remote_scene.stream','--source','gimbal-demo','--geometry','unprepared',
                     '--server-config',str(root/'server.json'),'--gimbal-control-config',str(root/'gimbal-server.json'),
                     '--port','0','--gimbal-port','0','--batches','60']
            process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8')
            try:
                start_line=process.stdout.readline()
                if not start_line:self.fail('CLI exited before listening: '+process.stderr.read())
                started=json.loads(start_line);self.assertTrue(started['retention_enabled'])
                scene['port']=started['port'];control['port']=started['simulated_control_port']
                with connect_pinned(scene,timeout=5.) as downlink,connect_pinned(control,timeout=5.,auth_magic=b'QGC1') as uplink:
                    first=unpack_packet(read_packet(downlink));initial=next(v.world_from_depth for v in first.views if v.camera_id==5 and not v.flags&1)
                    feedback=unpack_feedback(recv_exact(uplink,96));self.assertEqual(feedback['source_id'],first.source_id);self.assertNotIn(first.source_id,(0,SOURCE_ID,STATIC_SOURCE_ID))
                    sequence=0;client_stamp=0
                    def send(kind,clutch=False,pan=0.,lease=100):
                        nonlocal sequence,feedback,client_stamp
                        sequence+=1;client_stamp=max(client_stamp+1,time.monotonic_ns())
                        uplink.sendall(pack_command(Command(feedback['session'],feedback['challenge'],sequence,client_stamp,kind,clutch,pan,0.,lease)))
                        feedback=unpack_feedback(recv_exact(uplink,96));return feedback
                    send(HOLD)
                    for _ in range(6):send(AIM,True,.6,150);time.sleep(.06)
                    send(HOLD);self.assertGreater(feedback['pan'],.1)
                    deadline=time.monotonic()+4.;moved=None
                    while time.monotonic()<deadline:
                        candidate=unpack_packet(read_packet(downlink));camera=next(v for v in candidate.views if v.camera_id==5 and not v.flags&1)
                        if np.max(np.abs(camera.world_from_depth-initial))>.1:moved=candidate;break
                    self.assertIsNotNone(moved)
                    old_generation=moved.map_generation
                    # HOLD supplies a fresh challenge even after scene processing.
                    send(HOLD);send(RESET_MAP)
                    deadline=time.monotonic()+4.;reset=None
                    while time.monotonic()<deadline:
                        candidate=unpack_packet(read_packet(downlink))
                        if candidate.map_generation>old_generation:reset=candidate;break
                    self.assertIsNotNone(reset);self.assertFalse(any(v.flags&1 for v in reset.views))
                    send(HOLD);self.assertEqual(feedback['map_generation'],reset.map_generation)
                    before=feedback['pan'];send(AIM,True,.8,100);time.sleep(.19);send(HOLD)
                    self.assertFalse(feedback['flags']&ARMED);self.assertLessEqual(feedback['pan']-before,.105)
                    held=feedback['pan'];session=feedback['session']
                # Reconnecting creates a disarmed session at held measured angles.
                time.sleep(.05)
                with connect_pinned(control,timeout=5.,auth_magic=b'QGC1') as again:
                    reply=unpack_feedback(recv_exact(again,96));self.assertNotEqual(reply['session'],session)
                    self.assertFalse(reply['flags']&ARMED);self.assertAlmostEqual(reply['pan'],held,places=5)
                remaining,error=process.communicate(timeout=20)
                self.assertEqual(process.returncode,0,error)
                ended=json.loads(remaining.strip().splitlines()[-1]);self.assertEqual(ended['published'],60)
            except Exception as failure:
                if process.poll() is None:process.terminate()
                output,error=process.communicate(timeout=5)
                raise AssertionError(str(failure)+'; CLI stderr: '+error) from failure
            finally:
                if process.poll() is None:process.terminate();process.communicate(timeout=5)

if __name__=='__main__':unittest.main()
