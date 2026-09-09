from dataclasses import replace
from pathlib import Path
import socket
import tempfile
import time
import unittest
from tools.remote_scene.demo import make_demo
from tools.remote_scene.protocol import pack_packet,unpack_packet
from tools.remote_scene.provision import provision,DEFAULT_OPENSSL
from tools.remote_scene.publisher import LatestPublisher,connect_pinned,read_packet


@unittest.skipUnless(DEFAULT_OPENSSL.is_file(),'explicit OpenSSL executable required for temporary loopback certificate')
class PublisherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        cls.server,cls.client=provision(cls.temp.name,test_certificate=True)
        cls.packet,_=make_demo(16,12)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def publisher(self):
        s=self.server
        return LatestPublisher(s['certificate'],s['private_key'],bytes.fromhex(s['token_hex']))

    def config(self,publisher,**changes):return dict(self.client,port=publisher.address[1],**changes)

    def test_pinned_token_roundtrip_latest_only_and_sequence_guard(self):
        with self.publisher() as publisher:
            for sequence in range(1,5):publisher.publish(pack_packet(replace(self.packet,sequence=sequence)))
            with connect_pinned(self.config(publisher)) as client:
                self.assertEqual(unpack_packet(read_packet(client)).sequence,4)
                publisher.publish(pack_packet(replace(self.packet,sequence=5)))
                self.assertEqual(unpack_packet(read_packet(client)).sequence,5)
                with self.assertRaises(ValueError):publisher.publish(pack_packet(self.packet))
            self.assertEqual(publisher.version,5)

    def test_wrong_pin_and_token_fail_closed(self):
        with self.publisher() as publisher:
            publisher.publish(pack_packet(self.packet))
            with self.assertRaises(ValueError):connect_pinned(self.config(publisher,certificate_sha256='00'*32))
            with connect_pinned(self.config(publisher,token_hex='00'*32)) as client:
                with self.assertRaises((EOFError,ConnectionError)):read_packet(client)

    def test_shutdown_interrupts_stalled_handshake(self):
        publisher=self.publisher();publisher.start()
        stalled=socket.create_connection(publisher.address,timeout=1.)
        time.sleep(.03);start=time.monotonic();publisher.close();stalled.close()
        self.assertLess(time.monotonic()-start,1.)
        self.assertFalse(publisher.threads)

    def test_aba_and_history_overflow_keep_last_wire_unchanged(self):
        publisher=self.publisher()
        try:
            publisher.publish(pack_packet(replace(self.packet,sequence=10)))
            publisher.publish(pack_packet(replace(self.packet,world_epoch=2)))
            saved=publisher.wire;version=publisher.version
            with self.assertRaises(ValueError):publisher.publish(pack_packet(self.packet))
            self.assertEqual(publisher.wire,saved);self.assertEqual(publisher.version,version)
            for epoch in range(3,65):publisher.publish(pack_packet(replace(self.packet,world_epoch=epoch)))
            saved=publisher.wire;version=publisher.version
            with self.assertRaises(ValueError):publisher.publish(pack_packet(replace(self.packet,world_epoch=65)))
            self.assertEqual(publisher.wire,saved);self.assertEqual(publisher.version,version)
            publisher.publish(pack_packet(replace(self.packet,sequence=11)))
        finally:publisher.close()


if __name__=='__main__':unittest.main()
