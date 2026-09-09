"""Both scene representations use the same authenticated latest-only transport."""
from dataclasses import replace
import tempfile
import unittest
from tools.remote_scene.provision import provision,DEFAULT_OPENSSL
from tools.remote_scene.publisher import LatestPublisher,connect_pinned,read_packet
from tools.remote_scene.protocol import pack_packet,unpack_packet
from tools.remote_scene.surface_fixtures import make_fixtures


@unittest.skipUnless(DEFAULT_OPENSSL.is_file(),'explicit OpenSSL executable required for temporary loopback certificate')
class SurfaceTransportTests(unittest.TestCase):
    def test_both_surface_modes_share_one_authenticated_connection(self):
        with tempfile.TemporaryDirectory() as folder:
            server,client=provision(folder,test_certificate=True)
            prepared,raw=make_fixtures()['projective']
            with LatestPublisher(server['certificate'],server['private_key'],bytes.fromhex(server['token_hex'])) as publisher:
                publisher.publish(pack_packet(prepared))
                with connect_pinned(dict(client,port=publisher.address[1])) as connection:
                    first=unpack_packet(read_packet(connection));self.assertEqual(first.representation,1)
                    self.assertGreater(first.geometry_summary['triangles'],0)
                    publisher.publish(pack_packet(raw))
                    second=unpack_packet(read_packet(connection));self.assertEqual(second.representation,2)
                    self.assertEqual(len(second.views),1)
                    wire=publisher.wire
                    with self.assertRaises(ValueError):publisher.publish(pack_packet(prepared))
                    self.assertEqual(publisher.wire,wire)
                    publisher.publish(pack_packet(replace(prepared,sequence=2)))
                    third=unpack_packet(read_packet(connection));self.assertEqual(third.identity,first.identity)
                    self.assertEqual(third.sequence,2)


if __name__=='__main__':unittest.main()
