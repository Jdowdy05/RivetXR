"""Pure-Java transport checks; credentials and socket connections are loopback-only."""
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class RemoteWireTests(unittest.TestCase):
    def test_android_adapter_ownership_without_device(self):
        java,javac = (shutil.which(name) for name in ("java","javac"))
        self.assertTrue(java and javac,"Established JDK17 tools are required")
        fixture = ROOT / "tests/remote_scene_transport"
        app = ROOT / "quest/app/src/full/java/com/questnewton"
        with tempfile.TemporaryDirectory(prefix="remote-scene-adapter-") as directory:
            out=Path(directory);classes=out/"classes";classes.mkdir()
            sources=[app/"RemoteSceneWire.java",app/"RemoteSceneConnection.java",fixture/"RemoteSceneConnectionTest.java",
                     fixture/"stubs/android/content/Context.java",fixture/"stubs/android/content/res/AssetManager.java"]
            built=subprocess.run([javac,"-Xlint:all","-Werror","-d",str(classes),*map(str,sources)],capture_output=True,text=True,timeout=30)
            self.assertEqual(built.returncode,0,built.stdout+built.stderr)
            result=subprocess.run([java,"-cp",str(classes),"com.questnewton.RemoteSceneConnectionTest",str(out)],capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn("adapter constructor/worker/demo/config/close lifecycle passed",result.stdout)

    def test_wire_and_loopback_tls(self):
        java, javac, keytool = (shutil.which(name) for name in ("java", "javac", "keytool"))
        self.assertTrue(java and javac and keytool, "Established JDK17 tools are required")
        with tempfile.TemporaryDirectory(prefix="remote-scene-wire-") as directory:
            out = Path(directory)
            env = dict(os.environ, REMOTE_TEST_STORE_PASSWORD=secrets.token_hex(24))
            source = ROOT / "quest/app/src/full/java/com/questnewton/RemoteSceneWire.java"
            test = ROOT / "tests/remote_scene_transport/RemoteSceneWireTest.java"
            built=subprocess.run([javac,"-Xlint:all","-d",str(out),str(source),str(test)],capture_output=True,text=True,timeout=30)
            self.assertEqual(built.returncode,0,built.stdout+built.stderr)
            store = out / "fixture.p12"
            subprocess.run([keytool,"-genkeypair","-alias","remote-fixture","-keyalg","RSA","-keysize","2048",
                            "-validity","1","-dname","CN=Loopback Remote Scene Fixture","-storetype","PKCS12",
                            "-keystore",str(store),"-storepass:env","REMOTE_TEST_STORE_PASSWORD","-noprompt"],
                           check=True,capture_output=True,text=True,timeout=30,env=env)
            result = subprocess.run([java,"-cp",str(out),"com.questnewton.RemoteSceneWireTest",str(store)],
                                    check=False,capture_output=True,text=True,timeout=30,env=env)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn("loopback TLS pin/auth/receive-close/connect-close/deadlines passed",result.stdout)

    def test_gimbal_control_tls(self):
        java,javac,keytool=(shutil.which(name) for name in ('java','javac','keytool'))
        self.assertTrue(java and javac and keytool)
        with tempfile.TemporaryDirectory(prefix='gimbal-control-wire-') as directory:
            out=Path(directory);env=dict(os.environ,REMOTE_TEST_STORE_PASSWORD=secrets.token_hex(24))
            sources=[ROOT/'quest/app/src/full/java/com/questnewton/RemoteSceneWire.java',
                     ROOT/'quest/app/src/full/java/com/questnewton/GimbalControlConnection.java',
                     ROOT/'tests/remote_scene_transport/RemoteSceneWireTest.java',ROOT/'tests/remote_scene_transport/GimbalControlWireTest.java',
                     ROOT/'tests/remote_scene_transport/stubs/android/content/Context.java',ROOT/'tests/remote_scene_transport/stubs/android/content/res/AssetManager.java']
            built=subprocess.run([javac,'-Xlint:all','-d',str(out),*map(str,sources)],capture_output=True,text=True,timeout=30)
            self.assertEqual(built.returncode,0,built.stdout+built.stderr)
            store=out/'fixture.p12'
            subprocess.run([keytool,'-genkeypair','-alias','remote-fixture','-keyalg','RSA','-keysize','2048','-validity','1',
                '-dname','CN=Loopback Gimbal Fixture','-storetype','PKCS12','-keystore',str(store),'-storepass:env','REMOTE_TEST_STORE_PASSWORD','-noprompt'],
                check=True,capture_output=True,text=True,timeout=30,env=env)
            result=subprocess.run([java,'-cp',str(out),'com.questnewton.GimbalControlWireTest',str(store)],capture_output=True,text=True,timeout=15,env=env)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('control TLS auth, fixed framing, deadlines, single pending request and close passed',result.stdout)


if __name__ == "__main__":unittest.main()
