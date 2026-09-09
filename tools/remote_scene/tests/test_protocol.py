import unittest
from dataclasses import replace
import struct
import zlib
import numpy as np
from tools.remote_scene.protocol import Packet, POINT_DTYPE, pack_packet, unpack_packet,encode_transport,decode_transport,LatestPacket,HEADER


class PacketTests(unittest.TestCase):
    def packet(self):
        points=np.zeros(1,dtype=POINT_DTYPE);points['xyz']=[[1,2,3]];points['support']=1;points['radius_mm']=5
        return Packet(1,1,1,1,100,110,120,1,1,0,.01,20,(0,0,1,0,0,0,1),points)

    def test_exact_header_and_point_roundtrip(self):
        points=np.zeros(1,dtype=POINT_DTYPE)
        points['xyz']=[[1,2,3]];points['gray']=173;points['support']=2;points['radius_mm']=5
        packet=Packet(1,1,1,1,100,110,120,31,3,2,.01,20,(0,0,1,0,0,0,1),points)
        raw=pack_packet(packet)
        self.assertEqual(len(raw),144)
        decoded=unpack_packet(raw)
        self.assertEqual(decoded.source_id,1)
        np.testing.assert_array_equal(decoded.points,points)

    def test_crc_reserved_length_and_nonfinite_rejected(self):
        raw=pack_packet(self.packet())
        for offset in (0,4,6,92,124,128):
            corrupt=bytearray(raw);corrupt[offset]^=1
            with self.subTest(offset=offset),self.assertRaises(ValueError):unpack_packet(corrupt)
        for corrupt in (raw[:-1],raw+b'x'):
            with self.assertRaises(ValueError):unpack_packet(corrupt)
        packet=self.packet();packet.points['xyz'][0,0]=np.nan
        with self.assertRaises(ValueError):pack_packet(packet)

    def test_timestamps_masks_support_observer_and_limits(self):
        packet=self.packet()
        for change in (dict(source_id=0),dict(sequence=True),dict(contributing_mask=2),dict(flags=16),dict(flags=2),
                       dict(capture_end_ns=121),dict(capture_start_ns=0),dict(produced_ns=2100000000),
                       dict(voxel_size_m=.2),dict(observer_pose=(0,0,1,0,0,0,2))):
            with self.subTest(change=change),self.assertRaises(ValueError):pack_packet(replace(packet,**change))
        for field,value in (('support',2),('radius_mm',0),('radius_mm',51)):
            changed=self.packet();changed.points[field]=value
            with self.assertRaises(ValueError):pack_packet(changed)

    def test_bounded_zlib_and_sequence_identity_reset(self):
        raw=pack_packet(self.packet());wire=encode_transport(raw);size,compressed=struct.unpack('!II',wire[:8])
        self.assertEqual(decode_transport(size,wire[8:]),raw);self.assertEqual(compressed,len(wire)-8)
        for data in (wire[8:]+b'x',wire[8:-1],zlib.compress(b'x'*2000000)):
            with self.assertRaises(ValueError):decode_transport(size,data)
        latest=LatestPacket();latest.accept(raw)
        with self.assertRaises(ValueError):latest.accept(raw)
        latest.accept(pack_packet(replace(self.packet(),world_epoch=2)))

    def test_identity_history_rejects_aba_and_preserves_latest(self):
        latest=LatestPacket();packet=self.packet()
        latest.accept(pack_packet(replace(packet,sequence=10)))
        latest.accept(pack_packet(replace(packet,world_epoch=2)))
        with self.assertRaises(ValueError):latest.accept(pack_packet(packet))
        self.assertEqual(latest.packet.world_epoch,2)
        self.assertEqual(latest.packet.sequence,1)
        latest.accept(pack_packet(replace(packet,sequence=11)))

    def test_identity_history_bound_and_explicit_reset_match_native(self):
        latest=LatestPacket();packet=self.packet()
        for epoch in range(1,65):latest.accept(pack_packet(replace(packet,world_epoch=epoch)))
        with self.assertRaises(ValueError):latest.accept(pack_packet(replace(packet,world_epoch=65)))
        self.assertEqual(latest.packet.world_epoch,64)
        latest.accept(pack_packet(replace(packet,sequence=2)))
        latest.reset();self.assertIsNone(latest.packet)
        latest.accept(pack_packet(packet))


if __name__=='__main__':unittest.main()
