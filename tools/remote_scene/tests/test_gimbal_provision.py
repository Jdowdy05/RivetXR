import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from tools.remote_scene.provision import provision,DEFAULT_OPENSSL

@unittest.skipUnless(DEFAULT_OPENSSL.is_file(),'test certificate helper required')
class GimbalProvisionTests(unittest.TestCase):
    def test_optional_control_provisioning_uses_distinct_auth_and_same_pin(self):
        with tempfile.TemporaryDirectory() as directory:
            server,client=provision(directory,test_certificate=True,control_port=7444)
            control=json.loads((Path(directory)/'gimbal-server.json').read_text())
            control_client=json.loads((Path(directory)/'remote-gimbal-client.json').read_text())
            self.assertNotEqual(control['token_hex'],server['token_hex'])
            self.assertEqual(control_client['certificate_sha256'],client['certificate_sha256'])
            self.assertEqual(control_client['port'],7444);self.assertEqual(control_client['token_hex'],control['token_hex'])

    def test_bad_ports_or_repeated_tokens_never_write_complete_configs(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):provision(directory,test_certificate=True,control_port=7443)
            self.assertFalse(list(Path(directory).iterdir()))
        with tempfile.TemporaryDirectory() as directory,patch('tools.remote_scene.provision.secrets.token_hex',return_value='00'*32):
            with self.assertRaises(ValueError):provision(directory,test_certificate=True,control_port=7444)
            self.assertFalse((Path(directory)/'server.json').exists())

    def test_default_provisioning_does_not_add_control_credentials(self):
        with tempfile.TemporaryDirectory() as directory:
            provision(directory,test_certificate=True)
            self.assertFalse((Path(directory)/'gimbal-server.json').exists())

if __name__=='__main__':unittest.main()
