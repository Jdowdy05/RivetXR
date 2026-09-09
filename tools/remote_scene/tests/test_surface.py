import unittest
import struct
from dataclasses import replace
import numpy as np
from tools.remote_scene.calibration import CameraCalibration, DepthFrame, Intrinsics
from tools.remote_scene.protocol import pack_packet, unpack_packet, encode_transport, decode_transport, LatestPacket
from tools.remote_scene.surface import build_surface_batch, prepare_surface
from tools.remote_scene.surface_protocol import SurfacePacket, SurfaceView


def example(width=4, height=3, depth=None, camera_id=0, capture=100, world=None):
    intr=Intrinsics(width,height,10.,10.,(width-1)/2,(height-1)/2)
    camera=CameraCalibration(camera_id,intr,intr,.001,np.eye(4),np.eye(4))
    frame=DepthFrame(camera_id,1,capture,np.full((height,width),1000,dtype='<u2') if depth is None else depth,
                     np.arange(width*height,dtype='u1').reshape(height,width),world)
    return camera,frame


def make(camera=None,frame=None,geometry='prepared',**kwargs):
    if camera is None:camera,frame=example()
    return build_surface_batch([frame],[camera],geometry=geometry,produced_ns=200,rig_mode='stationary',**kwargs)[0]


class SurfaceTests(unittest.TestCase):
    def test_prepared_and_unprepared_canonical_roundtrip(self):
        p=make();raw=pack_packet(p);q=unpack_packet(raw)
        self.assertEqual(raw[:8],b'RSCN\x02\x00\xa0\x00')
        self.assertEqual(p.representation,1);self.assertEqual(p.vertices.shape,(12,6));self.assertEqual(len(p.indices),36)
        np.testing.assert_array_equal(q.vertices,p.vertices);np.testing.assert_array_equal(q.indices,p.indices)
        np.testing.assert_array_equal(q.atlas,p.atlas)
        raw_packet=unpack_packet(pack_packet(make(geometry='unprepared')))
        mesh=prepare_surface(raw_packet)
        np.testing.assert_array_equal(mesh.vertices,p.vertices);np.testing.assert_array_equal(mesh.indices,p.indices)
        np.testing.assert_array_equal(mesh.atlas,p.atlas);self.assertEqual(mesh.representation,2)
        wire=encode_transport(raw);self.assertEqual(decode_transport(len(raw),wire[8:]),raw)

    def test_hole_depth_edge_and_no_triangle_bridge(self):
        depths=np.full((4,6),1000,dtype='<u2');depths[:,3:]=1500;depths[1,1]=0
        c,f=example(6,4,depths);p=make(c,f)
        self.assertGreater(len(p.indices),0)
        for tri in p.vertices[p.indices.reshape(-1,3),:3]:
            self.assertLess(np.ptp(tri[:,2]),.051)
        self.assertFalse(np.any(np.all(np.isclose(p.vertices[:,:3],[-.15,-.05,1]),axis=1)))

    def test_empty_preserves_captured_metadata_and_no_camera_empty(self):
        c,f=example(depth=np.zeros((3,4),dtype='<u2'));p=make(c,f)
        self.assertEqual(p.vertices.shape,(0,6));self.assertEqual(p.atlas.shape,(0,0))
        self.assertEqual(unpack_packet(pack_packet(p)).contributing_mask,1)
        empty,_=build_surface_batch([], [c], geometry='unprepared',produced_ns=200,rig_mode='stationary')
        self.assertEqual(empty.capture_start_ns,0);self.assertEqual(len(pack_packet(empty)),160)
        self.assertEqual(prepare_surface(empty).atlas.size,0)

    def test_moving_exposure_pose_is_combined_before_float32_wire(self):
        pose=np.eye(4);pose[0,3]=.123456789
        c,f=example(world=pose)
        p,_=build_surface_batch([f],[c],geometry='unprepared',produced_ns=200,rig_mode='posed')
        self.assertEqual(p.views[0].world_from_depth[0,3],float(np.float32(pose[0,3])))
        mesh=prepare_surface(unpack_packet(pack_packet(p)))
        stationary=make()
        np.testing.assert_allclose(mesh.vertices[:,0], stationary.vertices[:,0]+np.float32(pose[0,3]),atol=2e-8)
        with self.assertRaises(ValueError):make(c,f)

    def test_reduction_is_bounded_and_matches_pixel_center_selection(self):
        c,f=example(128,96);p=make(c,f,geometry='unprepared')
        v=p.views[0];self.assertEqual(v.depth.shape,(48,64));self.assertEqual(v.intensity.shape,(96,128))
        self.assertEqual(v.depth_intrinsics,(5.,5.,31.5,23.5));self.assertTrue(p.flags&1)
        c,f=example(640,480);p=make(c,f,geometry='unprepared')
        np.testing.assert_array_equal(p.views[0].intensity,f.intensity[1::2,1::2])
        self.assertLess(len(pack_packet(p)),1024*1024)

    def test_projective_uvq_keeps_intensity_depth(self):
        c,f=example()
        transform=np.eye(4);transform[0,3]=.01;transform[2,3]=.1
        c=replace(c,intensity_from_depth=transform)
        p=make(c,f)
        self.assertTrue(np.allclose(p.vertices[:,5],1.1))
        xyz=p.vertices[:,:3].astype(float);q=xyz[:,2]+np.float32(.1)
        u=(10*(xyz[:,0]+np.float32(.01))/q+1.5+.5)/4
        np.testing.assert_allclose(p.vertices[:,3]/p.vertices[:,5],u,atol=1e-7)

    def test_identity_representation_and_rejected_publication(self):
        p=make();r=make(geometry='unprepared');latest=LatestPacket()
        latest.accept(pack_packet(p));latest.accept(pack_packet(r))
        self.assertNotEqual(p.identity,r.identity)
        with self.assertRaises(ValueError):latest.accept(pack_packet(p))
        self.assertEqual(latest.packet.representation,2)

    def test_packet_semantic_and_binary_validation(self):
        p=make()
        badvertices=p.vertices.copy();badvertices[0,5]=0
        for bad in (replace(p,vertices=badvertices),replace(p,indices=np.array([0,0,1],dtype='<u2')),
                    replace(p,indices=np.array([0,1,999],dtype='<u2')),replace(p,atlas=np.zeros((0,0),dtype='u1'))):
            with self.assertRaises(ValueError):pack_packet(bad)
        for original in (pack_packet(p),pack_packet(make(geometry='unprepared'))):
            for offset in (92,124,156,160):
                bad=bytearray(original);bad[offset]^=1
                with self.subTest(offset=offset),self.assertRaises(ValueError):unpack_packet(bad)
            with self.assertRaises(ValueError):unpack_packet(original+b'\0')
        r=make(geometry='unprepared');view=r.views[0];bad=replace(view,world_from_depth=np.zeros((3,4),dtype='<f4'))
        with self.assertRaises(ValueError):pack_packet(replace(r,views=(bad,)))

    def test_visibility_rejects_far_depth_correspondence_and_behind_camera(self):
        depths=np.full((4,6),1000,dtype='<u2');depths[:,:3]=800
        c,f=example(6,4,depths)
        c=replace(c,intensity=Intrinsics(6,4,.1,.1,2.,1.))
        p=make(c,f)
        self.assertGreater(len(p.vertices),0)
        self.assertTrue(np.allclose(p.vertices[:,2],.8))
        transform=np.eye(4);transform[2,3]=-2
        p=make(replace(c,intensity_from_depth=transform),f)
        self.assertEqual(len(p.indices),0)

    def test_view_order_masks_skew_staleness_and_rejection_are_explicit(self):
        c0,f0=example(camera_id=0,capture=1000000000)
        c2,f2=example(camera_id=2,capture=1001000000)
        kwargs=dict(produced_ns=1002000000,rig_mode='stationary',geometry='unprepared')
        p,_=build_surface_batch([f2,f0],[c2,c0],**kwargs)
        self.assertEqual([v.camera_id for v in p.views],[0,2]);self.assertEqual(p.contributing_mask,5)
        for frames,calibrations in (([f0,f0],[c0]),([f0],[c0,c0]),([f2],[c0])):
            with self.assertRaises(ValueError):build_surface_batch(frames,calibrations,**kwargs)
        late=replace(f0,capture_ns=970000000)
        p,_=build_surface_batch([late,f2],[c0,c2],**kwargs)
        self.assertEqual(p.contributing_mask,4);self.assertTrue(p.flags&2)
        p,_=build_surface_batch([f0],[c0],geometry='prepared',produced_ns=4000000000,rig_mode='stationary')
        self.assertEqual(p.contributing_mask,0)
        with self.assertRaises(ValueError):make(geometry='points')

    def test_mutable_input_does_not_change_decoded_publication(self):
        for mode in ('prepared','unprepared'):
            mutable=bytearray(pack_packet(make(geometry=mode)));packet=unpack_packet(mutable)
            saved=pack_packet(packet);mutable[-1]^=255
            self.assertEqual(pack_packet(packet),saved)
            array=packet.vertices if mode=='prepared' else packet.views[0].depth
            self.assertFalse(array.flags.writeable)

    def test_cross_language_fixture_pairs_are_bit_exact_in_python(self):
        from tools.remote_scene.surface_fixtures import make_fixtures
        for name,(prepared,raw) in make_fixtures().items():
            with self.subTest(fixture=name):
                decoded=prepare_surface(unpack_packet(pack_packet(raw)))
                np.testing.assert_array_equal(decoded.vertices,prepared.vertices)
                np.testing.assert_array_equal(decoded.indices,prepared.indices)
                np.testing.assert_array_equal(decoded.atlas,prepared.atlas)
                self.assertLessEqual(len(pack_packet(prepared)),973984)

    def test_semantic_corruptions_with_valid_crc_fail_before_publication(self):
        import zlib
        latest=LatestPacket();latest.accept(pack_packet(make()))
        original=latest.packet
        def changed(raw,offset,fmt,value):
            raw=bytearray(raw);struct.pack_into(fmt,raw,offset,value)
            struct.pack_into('<I',raw,88,zlib.crc32(raw[160:]));return raw
        prepared=pack_packet(make());raw=pack_packet(make(geometry='unprepared'))
        corruptions=[changed(prepared,64,'<I',1),changed(prepared,80,'<f',.1),
                     changed(prepared,128,'<I',3),changed(prepared,132,'<I',16385),
                     changed(prepared,136,'<I',4),changed(prepared,140,'<I',2049),
                     changed(prepared,148,'<I',0),changed(prepared,152,'<I',1),
                     changed(prepared,160,'<f',float('nan')),changed(prepared,172,'<f',-1),
                     changed(prepared,180,'<f',2048),changed(prepared,160+24*12,'<H',65535),
                     changed(raw,160,'<I',4),changed(raw,164,'<H',65),changed(raw,168,'<H',321),
                     changed(raw,172,'<I',1),changed(raw,176,'<f',0),changed(raw,208,'<f',0),
                     changed(raw,212,'<f',0),changed(raw,260,'<f',-1),changed(raw,308,'<I',1),
                     changed(raw,312,'<Q',201)]
        for number,corrupt in enumerate(corruptions):
            with self.subTest(case=number),self.assertRaises(ValueError):latest.accept(corrupt)
            self.assertIs(latest.packet,original)

    def test_five_full_resolution_views_respect_wire_budget(self):
        cameras=[];frames=[]
        for i in range(5):
            c,f=example(640,480,camera_id=i)
            c=replace(c,depth=replace(c.depth,fx=1000.,fy=1000.),intensity=replace(c.intensity,fx=1000.,fy=1000.))
            cameras.append(c);frames.append(f)
        for mode in ('prepared','unprepared'):
            p,_=build_surface_batch(frames,cameras,geometry=mode,produced_ns=200,rig_mode='stationary')
            encoded=pack_packet(p);decoded=unpack_packet(encoded)
            self.assertLessEqual(len(encoded),973984)
            if mode=='prepared':
                self.assertEqual(decoded.atlas.shape,(240,1600));self.assertLessEqual(len(decoded.vertices),16384)


    def test_public_prepare_canonicalizes_supplied_calibration_to_wire_float32(self):
        raw=make(geometry='unprepared');v=raw.views[0]
        world=v.world_from_depth.astype(float);world[0,3]=.123456789
        v=replace(v,world_from_depth=world,depth_units_m=.0010000000042,
                  depth_intrinsics=(10.00000003,10.,1.500000004,1.))
        raw=replace(raw,views=(v,))
        direct=prepare_surface(raw);roundtrip=prepare_surface(unpack_packet(pack_packet(raw)))
        np.testing.assert_array_equal(direct.vertices,roundtrip.vertices)
        np.testing.assert_array_equal(direct.indices,roundtrip.indices)

    def test_rigid_tolerance_requires_both_row_and_column_orthogonality(self):
        raw=make(geometry='unprepared');v=raw.views[0]
        angle=np.deg2rad(30)
        rotation=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        scale=np.diag([np.sqrt(1.0012),1,1])
        # Each matrix has one Gram error below tolerance and the other above.
        for matrix in (scale@rotation,rotation@scale):
            transform=np.column_stack((matrix,np.zeros(3))).astype('<f4')
            with self.subTest(matrix=matrix),self.assertRaises(ValueError):
                pack_packet(replace(raw,views=(replace(v,world_from_depth=transform),)))

    def test_out_of_map_foreground_still_occludes_intensity_correspondence(self):
        c,f=example(3,3,depth=np.tile(np.array([1020,1020,1000],dtype='<u2'),(3,1)))
        world=np.eye(4);world[0,3]=100
        c=replace(c,rig_from_depth=world,intensity=Intrinsics(3,3,.1,.1,1.,1.))
        self.assertEqual(len(make(c,f).indices),0)
        f.depth[:,2]=0
        self.assertGreater(len(make(c,f).indices),0)

    def test_v2_unused_voxel_field_requires_zero_bits_not_negative_zero(self):
        for geometry in ('prepared','unprepared'):
            raw=bytearray(pack_packet(make(geometry=geometry)))
            # Header fields are outside payload CRC; -0.0 has the numeric value
            # zero but violates the reserved all-zero wire contract.
            struct.pack_into('<I',raw,80,0x80000000)
            with self.subTest(geometry=geometry),self.assertRaises(ValueError):
                unpack_packet(raw)


if __name__=='__main__':unittest.main()
