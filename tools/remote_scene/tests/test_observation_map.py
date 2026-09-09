import unittest
import copy
from dataclasses import replace
import numpy as np
from tools.remote_scene.tests.test_rgb import batch
from tools.remote_scene.rgb_surface import canonical_rgb_view,build_rgb_batch,prepare_rgb
from tools.remote_scene.observation_map import RetainedObservationMap,_carve,_prioritize_color
from tools.remote_scene.protocol import pack_packet,unpack_packet


def view(stamp=1000000000,sequence=1,x=0,rgb=True):
    pose=np.eye(4);pose[0,3]=x;c,f=batch(stamp,sequence,pose,rgb)
    return canonical_rgb_view(f[0],c[0],'stationary')[0]


class ObservationMapTests(unittest.TestCase):
    def test_novelty_original_age_ttl_and_retained_only(self):
        cache=RetainedObservationMap(retention_ms=1000)
        first=view();cache.update([first],(1,1,1),1001000000)
        near=view(1100000000,2,x=.01);out=cache.update([near],(1,1,1),1101000000)
        self.assertFalse(any(v.flags&1 for v in out))
        far=view(1200000000,3,x=.2);out=cache.update([far],(1,1,1),1201000000)
        old=[v for v in out if v.flags&1];self.assertEqual(len(old),1);self.assertEqual(old[0].capture_ns,first.capture_ns)
        out=cache.update([],(1,1,1),1500000000);self.assertEqual(len(out),1);self.assertEqual(out[0].capture_ns,1000000000)
        self.assertEqual(cache.update([],(1,1,1),2100000000),())
        self.assertEqual(cache.map_generation,1)

    def test_fresh_free_space_carves_but_occlusion_or_unknown_does_not(self):
        old=view();current=view(1100000000,2)
        for depth,expect_carve in ((2000,True),(800,False),(0,False)):
            fresh=replace(current,depth=np.full_like(current.depth,depth))
            carved,n=_carve(old,[fresh])
            with self.subTest(depth=depth):
                self.assertEqual(n>0,expect_carve)
                if not expect_carve:np.testing.assert_array_equal(carved.depth,old.depth)
        # The immutable original observation was never modified.
        self.assertTrue(np.all(old.depth==1000))

    def test_current_gray_is_suppressed_only_where_retained_color_is_supported(self):
        gray=view(rgb=False);color=replace(view(),flags=3)
        output,count=_prioritize_color([gray],[color])
        self.assertGreater(count,0);self.assertNotEqual(output[0].triangle_keep,gray.triangle_keep)
        for invalid in (replace(color,flags=1),replace(color,depth=np.zeros_like(color.depth)),
                        replace(color,depth=np.full_like(color.depth,2000))):
            output,count=_prioritize_color([gray],[invalid]);self.assertEqual(count,0)
            self.assertEqual(output[0].triangle_keep,gray.triangle_keep)

    def test_color_replacement_must_outlive_gray_using_both_exposure_times(self):
        gray=view(rgb=False)
        # Even a newer RGB depth pose cannot extend the older image's lifetime.
        color=replace(view(1100000000,2),flags=3,image_capture_ns=900000000)
        output,count=_prioritize_color([gray],[color])
        self.assertEqual(count,0);self.assertEqual(output[0].triangle_keep,gray.triangle_keep)
        output,count=_prioritize_color([gray],[replace(color,image_capture_ns=1000000000)])
        self.assertGreater(count,0)

    def test_near_expiry_rgb_cannot_leave_holes_after_disconnect_in_either_mode(self):
        cache=RetainedObservationMap(retention_ms=1000)
        cache.update([view(1000000000,1)],(1,1,1),1001000000)
        cache.update([view(1100000000,2,.2)],(1,1,1),1101000000)
        cameras,frames=batch(1950000000,1,rgb=False)
        cameras+=batch()[0]
        prepared=[]
        for geometry in ('prepared','unprepared'):
            with self.subTest(geometry=geometry):
                independent=copy.deepcopy(cache)
                packet,_=build_rgb_batch(frames,cameras,geometry=geometry,produced_ns=1951000000,
                    rig_mode='stationary',observation_map=independent,retention_enabled=True,retention_ms=1000)
                self.assertEqual(independent.last_stats['suppressed_gray_triangles'],0)
                decoded=unpack_packet(pack_packet(packet));cooked=prepare_rgb(decoded)
                metadata=cooked.views if geometry=='prepared' else cooked.prepared_views
                gray,rgb=metadata
                # At 50 ms after receipt the RGB patch has expired. All 30 newer
                # gray triangles must still exist for their remaining 949 ms.
                self.assertEqual((gray.index_count,rgb.index_count),(90,90))
                self.assertGreater(packet.produced_ns+50000000-rgb.capture_ns,1000000000)
                self.assertLess(packet.produced_ns+50000000-gray.capture_ns,1000000000)
                prepared.append(cooked)
        self.assertEqual(len(prepared),2)
        np.testing.assert_array_equal(prepared[0].vertices,prepared[1].vertices)
        np.testing.assert_array_equal(prepared[0].indices,prepared[1].indices)

    def test_tracking_source_and_clear_reset_generation_and_history(self):
        cache=RetainedObservationMap();cache.update([view()],(1,1,1),1001000000)
        cache.update([view(1100000000,2,.2)],(1,1,1),1101000000)
        self.assertTrue(cache._retained)
        self.assertEqual(cache.update([],(1,1,1),1200000000,tracking_valid=False),())
        lost=cache.map_generation;cache.update([],(1,1,1),1300000000,tracking_valid=False)
        self.assertEqual(cache.map_generation,lost)
        cache.update([view(1400000000,3)],(1,2,1),1401000000)
        self.assertGreater(cache.map_generation,lost);self.assertFalse(cache._retained)
        cache.clear();self.assertFalse(cache._anchors)

    def test_reordered_observation_failure_is_transactional(self):
        cache=RetainedObservationMap();cache.update([view()],(1,1,1),1001000000)
        before=dict(cache._latest);anchors=dict(cache._anchors);generation=cache.map_generation
        with self.assertRaises(ValueError):cache.update([view(900000000,2,.2)],(1,1,1),1002000000)
        self.assertEqual(cache._latest,before);self.assertEqual(cache.map_generation,generation)
        self.assertIs(cache._anchors[5],anchors[5])

    def test_retained_budget_and_prepared_raw_receive_same_carving_masks(self):
        cache=RetainedObservationMap()
        for i in range(8):
            v=view(1000000000+i*100000000,i+1,x=.15*i)
            out=cache.update([v],(1,1,1),v.capture_ns+1000000)
            self.assertLessEqual(sum(bool(v.flags&1) for v in out),4)
        self.assertEqual(sum(bool(v.flags&1) for v in out),4)
        c,f=batch(1900000000,10);raw,_=build_rgb_batch(f,c,geometry='unprepared',produced_ns=1901000000,rig_mode='stationary',
                      observation_map=cache,retention_enabled=True)
        decoded=unpack_packet(pack_packet(raw));converted=prepare_rgb(decoded)
        direct=prepare_rgb(raw)
        np.testing.assert_array_equal(converted.vertices,direct.vertices)
        np.testing.assert_array_equal(converted.indices,direct.indices)
        for a,b in zip(raw.views,decoded.views):self.assertEqual(a.triangle_keep,b.triangle_keep)

    def test_backwards_production_clock_cannot_rejuvenate_cached_observations(self):
        cache=RetainedObservationMap();cache.update([view()],(1,1,1),1001000000)
        cache.update([view(1100000000,2,.2)],(1,1,1),1101000000)
        with self.assertRaises(ValueError):cache.update([],(1,1,1),1050000000)
        self.assertEqual(cache._latest[5],(1100000000,2))

    def test_invalid_packet_metadata_does_not_commit_map_identity(self):
        c,f=batch();cache=RetainedObservationMap()
        build_rgb_batch(f,c,produced_ns=1001000000,rig_mode='stationary',observation_map=cache,retention_enabled=True)
        generation=cache.map_generation;identity=cache._identity
        with self.assertRaises(ValueError):
            build_rgb_batch(f,c,produced_ns=1001000000,rig_mode='stationary',observation_map=cache,retention_enabled=True,
                            source_id=2,observer_pose=(float('nan'),0,0,0,0,0,1))
        self.assertEqual(cache.map_generation,generation);self.assertEqual(cache._identity,identity)

    def test_depth_only_retention_does_not_expire_from_unused_image_age(self):
        cache=RetainedObservationMap(retention_ms=1000)
        first=replace(view(),flags=0,image_capture_ns=1)
        cache.update([first],(1,1,1),1001000000)
        second=replace(view(1200000000,2,.2),flags=0,image_capture_ns=1)
        output=cache.update([second],(1,1,1),1201000000)
        self.assertEqual(len([v for v in output if v.flags&1]),1)

if __name__=='__main__':unittest.main()
