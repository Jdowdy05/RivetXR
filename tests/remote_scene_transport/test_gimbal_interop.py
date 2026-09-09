"""Actual Python simulation service -> Java TLS client -> native policy parity."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from tools.remote_scene.provision import provision,DEFAULT_OPENSSL
from tools.remote_scene.gimbal import SimulatedPanTilt
from tools.remote_scene.gimbal_server import SimulatedGimbalServer

ROOT=Path(__file__).resolve().parents[2]

@unittest.skipUnless(DEFAULT_OPENSSL.is_file(),'established OpenSSL fixture helper required')
class GimbalInteropTests(unittest.TestCase):
    def test_cold_tls_initialization_precedes_tcp_connect(self):
        java,javac=(shutil.which(name) for name in ('java','javac'));self.assertTrue(java and javac)
        with tempfile.TemporaryDirectory(prefix='gimbal-cold-tls-') as directory:
            out=Path(directory);server_config,client_config=provision(out/'tls',test_certificate=True)
            sources=[ROOT/'quest/app/src/full/java/com/questnewton/RemoteSceneWire.java',ROOT/'tests/remote_scene_transport/SlowTlsInitialization.java']
            built=subprocess.run([javac,'-Xlint:all','-Werror','-d',str(out),*map(str,sources)],capture_output=True,text=True,timeout=30)
            self.assertEqual(built.returncode,0,built.stdout+built.stderr)
            driver=SimulatedPanTilt()
            with SimulatedGimbalServer(driver,server_config['certificate'],server_config['private_key'],bytes.fromhex(server_config['token_hex']),source_id=77) as server:
                config=out/'client.json';config.write_text(json.dumps(dict(client_config,port=server.address[1])))
                run=subprocess.run([java,'-cp',str(out),'com.questnewton.SlowTlsInitialization',str(config)],capture_output=True,text=True,timeout=15)
                self.assertEqual(run.returncode,0,run.stdout+run.stderr)
                self.assertIn('leaves peer authentication budget intact',run.stdout)
            self.assertTrue(driver.feedback().command_stale)

    def test_python_java_native_control_interoperability(self):
        java,javac=(shutil.which(name) for name in ('java','javac'));self.assertTrue(java and javac)
        with tempfile.TemporaryDirectory(prefix='gimbal-python-java-') as directory:
            out=Path(directory);server_config,client_config=provision(out/'tls',test_certificate=True)
            sources=[ROOT/'quest/app/src/full/java/com/questnewton/RemoteSceneWire.java',ROOT/'tests/remote_scene_transport/GimbalPythonInterop.java']
            built=subprocess.run([javac,'-Xlint:all','-Werror','-d',str(out),*map(str,sources)],capture_output=True,text=True,timeout=30)
            self.assertEqual(built.returncode,0,built.stdout+built.stderr)
            for mode in ('good','bad_pin','bad_token'):
                driver=SimulatedPanTilt()
                with SimulatedGimbalServer(driver,server_config['certificate'],server_config['private_key'],bytes.fromhex(server_config['token_hex']),source_id=77) as server:
                    config=dict(client_config,port=server.address[1]);path=out/(mode+'.json')
                    if mode=='bad_pin':config['certificate_sha256']='00'*32
                    if mode=='bad_token':config['token_hex']='00'*32
                    path.write_text(json.dumps(config));transcript=out/'control-transcript.bin'
                    run=subprocess.run([java,'-cp',str(out),'com.questnewton.GimbalPythonInterop',str(path),'good' if mode=='good' else 'reject',str(transcript)],capture_output=True,text=True,timeout=15)
                    self.assertEqual(run.returncode,0,run.stdout+run.stderr)
                    if mode=='good':
                        self.assertIn('hold aim expiry release and disconnect passed',run.stdout)
                        native=Path(os.environ.get('GIMBAL_INTEROP_TEST',ROOT/'out/host/tests/Debug/gimbal_interop_test.exe'))
                        if native.is_file():
                            checked=subprocess.run([str(native),str(transcript)],capture_output=True,text=True,timeout=10)
                            self.assertEqual(checked.returncode,0,checked.stdout+checked.stderr)
                            self.assertIn('matched 7 Java commands',checked.stdout)
                    else:self.assertIn('bad control credentials rejected',run.stdout)
                self.assertTrue(driver.feedback().command_stale)

if __name__=='__main__':unittest.main()
