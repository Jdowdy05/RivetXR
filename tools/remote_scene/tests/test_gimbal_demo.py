import unittest
from dataclasses import replace
import numpy as np
from tools.remote_scene.gimbal_demo import optical_pose,make_gimbal_demo,GimbalDemoSource,SOURCE_ID,STATIC_SOURCE_ID
from tools.remote_scene.rgb_surface import prepare_rgb
from tools.remote_scene.protocol import pack_packet
from tools.remote_scene.stream import parse_args,retention_policy

class GimbalDemoTests(unittest.TestCase):
    def test_positive_pan_left_tilt_up_and_fixed_pivot(self):
        base=optical_pose(0,0);pan=optical_pose(.2,0);tilt=optical_pose(0,.2)
        np.testing.assert_array_equal(base[:3,3],pan[:3,3]);np.testing.assert_array_equal(base[:3,3],tilt[:3,3])
        self.assertGreater(pan[1,2],base[1,2]);self.assertGreater(tilt[2,2],base[2,2])

    def test_static_assets_use_actual_simulated_sweep_and_are_deterministic(self):
        p,stats=make_gimbal_demo();raw,_=make_gimbal_demo(geometry='unprepared');again,_=make_gimbal_demo()
        self.assertEqual(pack_packet(p),pack_packet(again))
        q=prepare_rgb(raw);np.testing.assert_array_equal(p.vertices,q.vertices);np.testing.assert_array_equal(p.indices,q.indices)
        np.testing.assert_array_equal(p.atlas,q.atlas)
        self.assertEqual(p.source_id,STATIC_SOURCE_ID);self.assertNotEqual(p.source_id,SOURCE_ID)
        self.assertEqual(p.contributing_mask,63);self.assertGreater(stats['retained_views'],0)
        self.assertGreaterEqual(stats['suppressed_gray_triangles'],0)

    def test_reset_request_is_nonblocking_and_serviced_by_publisher_owner(self):
        source=GimbalDemoSource();before=source.map_generation;source.request_clear()
        self.assertEqual(source.map_generation,before);self.assertTrue(source.clear_requested.is_set())
        source.service_clear();self.assertGreater(source.map_generation,before);self.assertFalse(source.clear_requested.is_set())

    def test_retention_defaults_preserve_legacy_and_explicit_zero(self):
        args=parse_args(['--source','demo','--server-config','unused.json'])
        self.assertEqual(retention_policy(args,False),(False,30000));self.assertEqual(retention_policy(args,True),(True,30000))
        args=parse_args(['--source','gimbal-demo','--server-config','unused.json','--retain-seconds','0'])
        self.assertEqual(retention_policy(args,True),(False,30000))

    def test_live_instances_have_distinct_source_ids_and_skip_reserved_ids(self):
        from unittest.mock import patch
        with patch('tools.remote_scene.gimbal_demo.secrets.randbits',side_effect=[0,STATIC_SOURCE_ID,SOURCE_ID,17,18]):
            first=GimbalDemoSource();second=GimbalDemoSource()
        self.assertEqual(first.source_id,17);self.assertEqual(second.source_id,18)
        self.assertNotEqual(first.source_id,second.source_id)

if __name__=='__main__':unittest.main()
