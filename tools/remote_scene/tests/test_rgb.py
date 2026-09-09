import unittest
from dataclasses import replace
import numpy as np
from tools.remote_scene.calibration import CameraCalibration,DepthFrame,Intrinsics
from tools.remote_scene.rgb_surface import build_rgb_batch,prepare_rgb
from tools.remote_scene.observation_map import RetainedObservationMap
from tools.remote_scene.protocol import pack_packet,unpack_packet


def batch(stamp=1000000000,sequence=1,pose=None,rgb=True):
    k=Intrinsics(6,4,10.,10.,2.5,1.5);p=np.eye(4) if pose is None else pose
    c=CameraCalibration(5 if rgb else 0,k,k,.001,p,np.eye(4),image_format='RGB8' if rgb else 'Y8')
    image=np.zeros((4,6,3),dtype='u1') if rgb else np.full((4,6),100,dtype='u1')
    if rgb:image[:,:,0]=200;image[:,:,1]=np.arange(6)*30
    f=DepthFrame(c.camera_id,sequence,stamp,np.full((4,6),1000,dtype='<u2'),image)
    return [c],[f]


def build(cameras=None,frames=None,geometry='prepared',**kwargs):
    if cameras is None:cameras,frames=batch()
    return build_rgb_batch(frames,cameras,geometry=geometry,produced_ns=max([f.capture_ns for f in frames]+[1000000000])+1000000,rig_mode='stationary',**kwargs)[0]


class RGBTests(unittest.TestCase):
    def test_rgb_and_raw_match_and_preserve_color(self):
        p=build();self.assertEqual(p.vertices.shape,(24,11));self.assertEqual(p.atlas.shape,(4,6,3))
        raw=pack_packet(p);self.assertEqual(raw[:8],b'RSCN\x03\x00\xe0\x00')
        q=prepare_rgb(unpack_packet(pack_packet(build(geometry='unprepared'))))
        np.testing.assert_array_equal(p.vertices,q.vertices);np.testing.assert_array_equal(p.indices,q.indices)
        np.testing.assert_array_equal(p.atlas,q.atlas)

    def test_rgb_field_of_view_does_not_delete_depth_geometry(self):
        cameras,frames=batch();c=replace(cameras[0],intensity=Intrinsics(6,4,40.,40.,2.5,1.5))
        p=build([c],frames)
        self.assertEqual(len(p.vertices),24);self.assertEqual(len(p.indices),90)
        self.assertTrue(np.all(p.vertices[:,10]==1));self.assertTrue(np.any(p.vertices[:,3]/p.vertices[:,5]>1))
        p=build([c],[replace(frames[0],image_usable=False)])
        self.assertTrue(np.all(p.vertices[:,10]==0));self.assertEqual(len(p.vertices),24)

    def test_cache_tracking_and_clear_retire_identity(self):
        cache=RetainedObservationMap();c,f=batch();p=build(c,f,observation_map=cache,retention_enabled=True)
        original=p.map_generation;cache.clear();p=build(c,f,observation_map=cache,retention_enabled=True)
        self.assertGreater(p.map_generation,original)
        p=build(c,f,observation_map=cache,retention_enabled=True,tracking_valid=False)
        self.assertFalse(p.views);self.assertEqual(len(p.vertices),0)

    def test_fixture_pairs_are_canonical_and_large_packets_use_v3_transport(self):
        from tools.remote_scene.rgb_fixtures import make_rgb_fixtures
        from tools.remote_scene.protocol import encode_transport,decode_transport
        for name,(prepared,raw) in make_rgb_fixtures().items():
            with self.subTest(name=name):
                p=unpack_packet(pack_packet(prepared));q=prepare_rgb(unpack_packet(pack_packet(raw)))
                np.testing.assert_array_equal(p.vertices,q.vertices);np.testing.assert_array_equal(p.indices,q.indices)
                np.testing.assert_array_equal(p.atlas,q.atlas)
                encoded=encode_transport(pack_packet(p));self.assertEqual(decode_transport(len(pack_packet(p)),encoded[8:]),pack_packet(p))
        self.assertGreater(len(pack_packet(make_rgb_fixtures()['six_rgb'][0])),1024*1024)

    def test_image_occlusion_keeps_geometry_and_actual_uvq(self):
        c,f=batch();c=[replace(c[0],intensity=Intrinsics(6,4,.1,.1,2.,1.))]
        depth=f[0].depth.copy();depth[:,:3]=800;f=[replace(f[0],depth=depth)]
        p=build(c,f)
        self.assertGreater(np.count_nonzero(p.vertices[:,10]==0),0)
        self.assertGreater(np.count_nonzero(p.vertices[:,10]==1),0)
        self.assertTrue(np.any((p.vertices[:,2]> .99)&(p.vertices[:,10]==0)))
        self.assertTrue(np.all(p.vertices[:,5]>0))

    def test_calibration_and_image_exposure_survive_raw_wire(self):
        c,f=batch();pose=np.eye(4);pose[0,3]=.123456789
        f=[replace(f[0],image_capture_ns=f[0].capture_ns-1000000,image_from_depth=pose)]
        p=build(c,f,geometry='unprepared');q=unpack_packet(pack_packet(p))
        self.assertEqual(q.views[0].image_capture_ns,999000000)
        self.assertEqual(q.views[0].intensity_from_depth[0,3],np.float32(.123456789))
        with self.assertRaises(ValueError):build(c,[replace(f[0],image_capture_ns=2000000000)])

    def test_mutable_wire_does_not_mutate_owned_decoded_images(self):
        raw=bytearray(pack_packet(build(geometry='unprepared')));packet=unpack_packet(raw)
        saved=pack_packet(packet);raw[-1]^=1
        self.assertEqual(saved,pack_packet(packet));self.assertFalse(packet.views[0].intensity.flags.writeable)

    def test_prepared_metadata_bounds_are_exact_wire_values(self):
        p=build();v=p.vertices.copy();v[0,6]+=np.float32(1e-7)
        with self.assertRaises(ValueError):pack_packet(replace(p,vertices=v))

    def test_malformed_header_metadata_arrays_and_masks_fail_with_valid_crc(self):
        import struct,zlib
        from tools.remote_scene.protocol import LatestPacket
        current=LatestPacket();current.accept(pack_packet(build()));saved=current.packet
        def corrupt(raw,offset,fmt,value):
            data=bytearray(raw);struct.pack_into(fmt,data,offset,value);struct.pack_into('<I',data,88,zlib.crc32(data[224:]));return data
        p=pack_packet(build());r=pack_packet(build(geometry='unprepared'))
        examples=[corrupt(p,80,'<I',0x80000000),corrupt(p,160,'<Q',0),corrupt(p,168,'<Q',1),
            corrupt(p,176,'<I',0),corrupt(p,184,'<I',60001),corrupt(p,188,'<I',0),corrupt(p,192,'<I',40),
            corrupt(p,196,'<I',1),corrupt(p,200,'<I',208),corrupt(p,208,'<Q',1),
            corrupt(p,224,'<I',6),corrupt(p,228,'<I',4),corrupt(p,232,'<Q',0),corrupt(p,248,'<Q',2000000000),
            corrupt(p,256,'<I',1),corrupt(p,264,'<I',1),corrupt(p,272,'<I',1),corrupt(p,288,'<I',2),
            corrupt(p,292,'<I',1),corrupt(p,304,'<f',float('nan')),corrupt(p,304+40,'<f',.5),
            corrupt(r,224+184,'<I',1),corrupt(r,224+180,'<I',2),corrupt(r,224+188,'<I',1),
            corrupt(r,224+160,'<Q',0),corrupt(r,224+168,'<Q',0)]
        # 30 candidate triangles have two unused mask bits in their final byte.
        bad=bytearray(r);bad[-1]|=0x80;struct.pack_into('<I',bad,88,zlib.crc32(bad[224:]));examples.append(bad)
        for index,bad in enumerate(examples):
            with self.subTest(index=index),self.assertRaises(ValueError):current.accept(bad)
            self.assertIs(current.packet,saved)

    def test_legacy_builders_explicitly_reject_sixth_or_rgb(self):
        from tools.remote_scene.fusion import fuse_batch
        from tools.remote_scene.surface import build_surface_batch
        c,f=batch()
        for fn in (fuse_batch,build_surface_batch):
            with self.assertRaises(ValueError):fn(f,c,produced_ns=1001000000,rig_mode='stationary')

    def test_old_unusable_image_keeps_fresh_depth_and_uses_depth_age(self):
        c,f=batch(stamp=5000000000)
        f=[replace(f[0],image_capture_ns=1000000000,image_usable=False)]
        p=build(c,f,geometry='unprepared')
        self.assertEqual(len(p.views),1);self.assertEqual(p.oldest_observation_ns,5000000000)
        self.assertEqual(p.views[0].image_capture_ns,1000000000)
        self.assertGreater(len(prepare_rgb(unpack_packet(pack_packet(p))).vertices),0)
        p=build(c,[replace(f[0],image_usable=True)],geometry='unprepared')
        self.assertEqual(len(p.views),0)

if __name__=='__main__':unittest.main()
