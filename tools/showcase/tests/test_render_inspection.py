import unittest
import numpy as np
from tools.showcase.render_inspection import box_hit,cylinder_hit,build_scene_packets,cast_scene

class InspectionShowcaseTests(unittest.TestCase):
    def test_known_geometry_generates_metric_depth(self):
        origin=np.array([0.,0.,0.]);directions=np.array([[0.,0.,1.],[2.,0.,1.]])
        t,n=box_hit(origin,directions,np.array([-.5,-.5,2.]),np.array([.5,.5,3.]))
        self.assertAlmostEqual(t[0],2.);self.assertTrue(np.isinf(t[1]));np.testing.assert_array_equal(n[0],[0,0,-1])
        t,n=cylinder_hit(origin,directions,np.array([0.,-.5,3.]),np.array([0.,.5,3.]),.5)
        self.assertAlmostEqual(t[0],2.5);self.assertTrue(np.isinf(t[1]))

    def test_scene_roundtrips_through_both_production_representations(self):
        texture=np.full((32,32,3),180,dtype=np.uint8)
        packet,raw_packet,inputs,proof=build_scene_packets(texture)
        self.assertTrue(proof['prepared_unprepared_equal']);self.assertEqual(packet.contributing_mask,63)
        self.assertGreater(len(packet.vertices),1000);self.assertEqual(len(inputs),6)
        self.assertFalse(np.shares_memory(packet.vertices,raw_packet.vertices))

    def test_albedo_changes_rgb_but_never_depth(self):
        origin=np.array([-.8,-.4,3.]);rays=np.array([[0.,0.,-1.]])
        red=np.full((8,8,3),[200,20,20],dtype=np.uint8)
        blue=np.full((8,8,3),[20,20,200],dtype=np.uint8)
        depth_a,rgb_a=cast_scene(origin,rays,red,shadows=False)
        depth_b,rgb_b=cast_scene(origin,rays,blue,shadows=False)
        np.testing.assert_array_equal(depth_a,depth_b)
        self.assertAlmostEqual(depth_a[0],1.92)
        self.assertFalse(np.array_equal(rgb_a,rgb_b))

if __name__=='__main__':unittest.main()
