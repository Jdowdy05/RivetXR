import tempfile
import time
import unittest
from tools.remote_scene.provision import provision,DEFAULT_OPENSSL
from tools.remote_scene.publisher import connect_pinned,recv_exact
from tools.remote_scene.gimbal import SimulatedPanTilt
from tools.remote_scene.gimbal_server import SimulatedGimbalServer
from tools.remote_scene.gimbal_control import Command,pack_command,unpack_feedback,HOLD,AIM,RESET_MAP,ARMED


@unittest.skipUnless(DEFAULT_OPENSSL.is_file(),'test certificate helper required')
class GimbalServerTests(unittest.TestCase):
    def test_pinned_simulator_lease_reset_and_disconnect(self):
        with tempfile.TemporaryDirectory() as directory:
            config,client=provision(directory,test_certificate=True);resets=[]
            driver=SimulatedPanTilt()
            with SimulatedGimbalServer(driver,config['certificate'],config['private_key'],bytes.fromhex(config['token_hex']),
                    reset_map=lambda:resets.append(True),source_id=77) as server:
                with connect_pinned(dict(client,port=server.address[1]),auth_magic=b'QGC1') as connection:
                    feedback=unpack_feedback(recv_exact(connection,96));self.assertEqual(feedback['source_id'],77)
                    for sequence,kind,clutch in ((1,HOLD,False),(2,AIM,True)):
                        connection.sendall(pack_command(Command(feedback['session'],feedback['challenge'],sequence,sequence,kind,clutch,.5,0.,100)))
                        feedback=unpack_feedback(recv_exact(connection,96))
                    self.assertTrue(feedback['flags']&ARMED)
                    time.sleep(.17)
                    self.assertLessEqual(server.sample().pan_rad,.101)
                    connection.sendall(pack_command(Command(feedback['session'],feedback['challenge'],3,3,RESET_MAP)))
                    feedback=unpack_feedback(recv_exact(connection,96))
                    self.assertFalse(feedback['flags']&ARMED);self.assertEqual(resets,[True])
                time.sleep(.02)
                self.assertFalse(server.session and server.session.armed)
            self.assertTrue(driver.feedback().command_stale)


if __name__=='__main__':unittest.main()
