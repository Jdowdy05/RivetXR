"""Measured Panda pad geometry must retain original mass, meshes and topology."""
import json
import inspect
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'quest/app/src/main/python'))
from quest_sim.finger_pads import add_finger_pads


@unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'real Panda assets required')
class FingerPadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from quest_sim.runtime import Session
        import newton
        import numpy as np
        import warp as wp
        cls.np, cls.wp, cls.newton = np, wp, newton
        settings = json.dumps(dict(initial_base_pose=[0, 0, 1.2, 0, 0, 0, 1]))
        # Root integration enables pads by default. Keep this component test's
        # baseline mesh-only and inject only geometry, without material tuning.
        profile = {'contact_profile': 'legacy_mesh_v1'} if 'contact_profile' in inspect.signature(Session).parameters else {}
        cls.baseline = Session(os.environ['FRANKA_DESCRIPTION_ROOT'], settings, **profile)
        finalize = newton.ModelBuilder.finalize

        def with_pads(builder, *args, **kwargs):
            cls.builder = builder
            cls.body_labels, cls.joint_labels = list(builder.body_label), list(builder.joint_label)
            cls.mass_before = np.asarray(builder.body_mass).copy()
            cls.com_before = np.asarray(builder.body_com).copy()
            cls.inertia_before = np.asarray(builder.body_inertia).copy()
            cls.original_shapes = [(body, kind, source, tuple(pose), tuple(scale), flags)
                                   for body, kind, source, pose, scale, flags in zip(
                                       builder.shape_body, builder.shape_type, builder.shape_source,
                                       builder.shape_transform, builder.shape_scale, builder.shape_flags)]
            cls.pad_indices = add_finger_pads(builder)
            return finalize(builder, *args, **kwargs)

        with mock.patch.object(newton.ModelBuilder, 'finalize', with_pads):
            cls.padded = Session(os.environ['FRANKA_DESCRIPTION_ROOT'], settings, **profile)

    def test_ten_boxes_preserve_builder_and_native_inertial_state(self):
        np, old, new = self.np, self.baseline, self.padded
        self.assertEqual(len(self.pad_indices), 10)
        self.assertEqual(new.model.shape_count, old.model.shape_count+10)
        self.assertEqual(self.builder.body_label, self.body_labels)
        self.assertEqual(self.builder.joint_label, self.joint_labels)
        for expected, current in ((self.mass_before, self.builder.body_mass),
                                  (self.com_before, self.builder.body_com),
                                  (self.inertia_before, self.builder.body_inertia)):
            np.testing.assert_array_equal(expected, current)
        for name in ('body_mass', 'body_inv_mass', 'body_com', 'body_inertia', 'body_inv_inertia'):
            np.testing.assert_array_equal(getattr(old.model, name).numpy(), getattr(new.model, name).numpy())
        a, b = old.solver.mj_model, new.solver.mj_model
        self.assertEqual(b.ngeom, a.ngeom+10)
        for name in ('nbody', 'njnt', 'nq', 'nv', 'nu', 'neq'):
            self.assertEqual(getattr(a, name), getattr(b, name))
        for name in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat'):
            np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
        np.testing.assert_array_equal(old.state.body_q.numpy(), new.state.body_q.numpy())

    def test_meshes_retained_and_native_pads_are_collidable_boxes(self):
        b, s, np = self.builder, self.padded.solver, self.np
        for i, (body, kind, source, pose, scale, flags) in enumerate(self.original_shapes):
            self.assertEqual((b.shape_body[i], b.shape_type[i], tuple(b.shape_transform[i]),
                              tuple(b.shape_scale[i]), b.shape_flags[i]), (body, kind, pose, scale, flags))
            self.assertIs(b.shape_source[i], source)
        for i, shape in enumerate(self.pad_indices):
            body = b.body_label.index('panda_leftfinger' if i<5 else 'panda_rightfinger')
            self.assertEqual(b.shape_body[shape], body)
            self.assertEqual(b.shape_type[shape], self.newton.GeoType.BOX)
            self.assertTrue(b.shape_flags[shape] & self.newton.ShapeFlags.COLLIDE_SHAPES)
            self.assertFalse(b.shape_flags[shape] & self.newton.ShapeFlags.SITE)
            geom = s._mujoco.mj_name2id(s.mj_model, s._mujoco.mjtObj.mjOBJ_GEOM, f'{b.shape_label[shape]}_{shape}')
            self.assertGreaterEqual(geom, 0)
            self.assertEqual(s.mj_model.geom_type[geom], s._mujoco.mjtGeom.mjGEOM_BOX)
            self.assertNotEqual(int(s.mj_model.geom_contype[geom]) | int(s.mj_model.geom_conaffinity[geom]), 0)
            self.assertEqual(s.mjc_body_to_newton.numpy()[0, s.mj_model.geom_bodyid[geom]], body)
            np.testing.assert_allclose(s.mj_model.geom_size[geom], [.008, .0005, .0012], rtol=0, atol=1e-9)

    def test_facing_patch_uses_the_existing_mesh_transform_and_fits_flat_region(self):
        b, wp, np = self.builder, self.wp, self.np
        centres = (.0375, .0403, .0431, .0459, .0487)
        for i, shape in enumerate(self.pad_indices):
            body = b.shape_body[shape]
            mesh = next(j for j, value in enumerate(self.original_shapes)
                        if value[0]==body and value[1]==self.newton.GeoType.MESH)
            local = wp.transform_multiply(wp.transform_inverse(b.shape_transform[mesh]), b.shape_transform[shape])
            np.testing.assert_allclose(tuple(local), [0, .0005, centres[i%5], 0, 0, 0, 1], rtol=0, atol=1e-8)
            source = b.shape_source[mesh].vertices
            front = source[np.isclose(source[:, 1], source[:, 1].min(), rtol=0, atol=1e-7)]
            self.assertTrue(0 < source[:, 1].min() < .0001)  # Tiny outset to plane y=0.
            self.assertGreater(-.008, float(front[:, 0].min()))
            self.assertLess(.008, float(front[:, 0].max()))
            self.assertGreater(centres[i%5]-.0012, float(front[:, 2].min()))
            self.assertLess(centres[i%5]+.0012, .0502087)  # Below the measured tip taper.
            self.assertAlmostEqual(float(b.shape_transform[shape].p[1]), .0005 if i<5 else -.0005, places=8)
        # Adjacent pads have 0.4 mm gaps and opposing faces meet at zero joint opening.
        np.testing.assert_allclose(np.diff(centres)-.0024, [.0004]*4, rtol=0, atol=1e-12)

    def test_duplicate_application_and_missing_finger_reject_before_adding_shapes(self):
        before = len(self.builder.shape_body)
        with self.assertRaises(ValueError):
            add_finger_pads(self.builder)
        self.assertEqual(len(self.builder.shape_body), before)
        b = self.newton.ModelBuilder()
        b.add_body(label='panda_leftfinger')
        with self.assertRaises(ValueError):
            add_finger_pads(b)
        self.assertEqual(len(b.shape_body), 0)


if __name__ == '__main__':
    unittest.main()
