import tempfile
import unittest
from dataclasses import replace
from tools.remote_scene.provision import provision,DEFAULT_OPENSSL
from tools.remote_scene.publisher import LatestPublisher,connect_pinned,read_packet
from tools.remote_scene.rgb_fixtures import make_rgb_fixtures
from tools.remote_scene.protocol import pack_packet,unpack_packet

@unittest.skipUnless(DEFAULT_OPENSSL.is_file(),'explicit OpenSSL required for temporary loopback certificate')
class RGBTransportTests(unittest.TestCase):
    def test_v3_rgb_raw_and_map_clear_use_one_authenticated_stream(self):
        with tempfile.TemporaryDirectory() as folder:
            server,client=provision(folder,test_certificate=True)
            prepared,raw=make_rgb_fixtures()['ten_views']
            with LatestPublisher(server['certificate'],server['private_key'],bytes.fromhex(server['token_hex'])) as publisher:
                publisher.publish(pack_packet(prepared))
                with connect_pinned(dict(client,port=publisher.address[1])) as connection:
                    decoded=unpack_packet(read_packet(connection))
                    self.assertEqual(len(decoded.views),10);self.assertEqual(decoded.map_generation,1)
                    publisher.publish(pack_packet(raw));decoded=unpack_packet(read_packet(connection))
                    self.assertEqual(decoded.representation,2);self.assertEqual(len(decoded.views),10)
                    cleared=replace(prepared,map_generation=2)
                    publisher.publish(pack_packet(cleared));decoded=unpack_packet(read_packet(connection))
                    self.assertEqual(decoded.map_generation,2)
                    before=publisher.wire
                    with self.assertRaises(ValueError):publisher.publish(pack_packet(prepared))
                    self.assertEqual(before,publisher.wire)

if __name__=='__main__':unittest.main()
