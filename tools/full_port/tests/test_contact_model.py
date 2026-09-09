"""Native model contracts for the finger-pad manipulation profile."""
import json
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'quest/app/src/main/python'))
from quest_sim.runtime import Session


@unittest.skipUnless(os.environ.get('FRANKA_DESCRIPTION_ROOT'), 'real Franka assets required')
class ContactModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.legacy = Session(os.environ['FRANKA_DESCRIPTION_ROOT'], '{}', contact_profile='legacy_mesh_v1')
        cls.current = Session(os.environ['FRANKA_DESCRIPTION_ROOT'], '{}', contact_profile='pad_manipulation_v1')

    def finger_geoms(self, s):
        mj, m = s.solver._mujoco, s.solver.mj_model
        for name in ('panda_finger_joint1', 'panda_finger_joint2'):
            joint = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, name)
            body = int(m.jnt_bodyid[joint])
            yield s.np.flatnonzero(m.geom_bodyid == body)

    def test_five_extra_boxes_and_retained_mesh_per_finger(self):
        old, new = self.legacy, self.current
        mj = new.solver._mujoco
        for before, after in zip(self.finger_geoms(old), self.finger_geoms(new)):
            old_types = old.solver.mj_model.geom_type[before]
            types = new.solver.mj_model.geom_type[after]
            self.assertEqual(sum(types == mj.mjtGeom.mjGEOM_BOX) - sum(old_types == mj.mjtGeom.mjGEOM_BOX), 5)
            self.assertEqual(sum(types == mj.mjtGeom.mjGEOM_MESH), sum(old_types == mj.mjtGeom.mjGEOM_MESH))
            self.assertGreater(sum(types == mj.mjtGeom.mjGEOM_MESH), 0)
        self.assertEqual(new.solver.mj_model.ngeom, old.solver.mj_model.ngeom + 10)

    def test_native_friction_and_contact_options(self):
        s = self.current; m = s.solver.mj_model; mj = s.solver._mujoco
        self.assertEqual(int(m.opt.cone), int(mj.mjtCone.mjCONE_ELLIPTIC))
        self.assertEqual(m.opt.impratio, 10.)
        self.assertFalse(int(m.opt.disableflags) & int(mj.mjtDisableBit.mjDSBL_MULTICCD))
        for indices in self.finger_geoms(s):
            s.np.testing.assert_array_equal(m.geom_condim[indices], 4)
            s.np.testing.assert_allclose(m.geom_friction[indices, 0], 1.5, rtol=0, atol=1e-8)
            s.np.testing.assert_allclose(m.geom_friction[indices, 1], .005, rtol=0, atol=1e-8)
        old_fingers = tuple(self.finger_geoms(self.legacy))
        for before, after in zip(old_fingers, self.finger_geoms(s)):
            for name in ('geom_solref', 'geom_solimp'):
                for row in getattr(m, name)[after]:
                    s.np.testing.assert_array_equal(row, getattr(self.legacy.solver.mj_model, name)[before[0]])
        meta = json.loads(s.metadata())
        self.assertEqual(meta['contact_model']['profile'], 'pad_manipulation_v1')

    def test_articulation_mass_inertia_and_actuation_unchanged(self):
        old, new = self.legacy.solver.mj_model, self.current.solver.mj_model
        self.assertEqual((old.nbody, old.njnt, old.neq), (new.nbody, new.njnt, new.neq))
        for name in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'jnt_range',
                     'jnt_actfrcrange', 'actuator_gainprm', 'actuator_biasprm', 'eq_data'):
            with self.subTest(name=name):
                self.current.np.testing.assert_array_equal(getattr(old, name), getattr(new, name))
        self.assertEqual(old.opt.integrator, new.opt.integrator)
        self.assertEqual(old.opt.iterations, new.opt.iterations)

    def test_unknown_profile_fails_explicitly(self):
        with self.assertRaises(ValueError):
            Session(os.environ['FRANKA_DESCRIPTION_ROOT'], '{}', contact_profile='typo')

    def test_default_and_explicit_setting_transition_reset_with_validated_profile(self):
        s = Session(os.environ['FRANKA_DESCRIPTION_ROOT'], '{}')
        self.assertEqual(json.loads(s.metadata())['contact_model']['profile'], 'legacy_mesh_v1')
        s.step(.005, [0, -.569, 0, -2.81, 0, 3.037, .741], 0.)
        s.configure('{"contact_profile":"pad_manipulation_v1"}')
        self.assertEqual(s.step_index, 0)
        self.assertEqual(s.settings['contact_profile'], 'pad_manipulation_v1')
        self.assertEqual(json.loads(s.metadata())['contact_model']['profile'], 'pad_manipulation_v1')
        model, settings = s.model, dict(s.settings)
        with self.assertRaises(ValueError):
            s.configure('{"contact_profile":"unknown"}')
        self.assertIs(s.model, model)
        self.assertEqual(s.settings, settings)
        with self.assertRaises(ValueError):
            Session(os.environ['FRANKA_DESCRIPTION_ROOT'], '{"contact_profile":"five_pads_v1"}',
                    contact_profile='legacy_mesh_v1')

    def test_profile_survives_object_room_and_floating_root_rebuilds(self):
        for name in ('legacy_mesh_v1', 'five_pads_v1', 'pad_manipulation_v1'):
            with self.subTest(profile=name):
                s = Session(os.environ['FRANKA_DESCRIPTION_ROOT'], '{}', contact_profile=name)
                expected = json.loads(s.metadata())['contact_model']
                s.command(json.dumps(dict(version=1, op='spawn', id=1,
                    pose=[2, 0, 1, 0, 0, 0, 1], half_extents=[.025]*3)))
                s.set_environment(json.dumps(dict(version=1, revision=1, enabled=True,
                    colliders=[dict(kind='floor', pose=[0, 0, -.025, 0, 0, 0, 1], half_extents=[3,3,.025])])))
                s.configure('{"floating_base":true}')
                self.assertEqual(json.loads(s.metadata())['contact_model'], expected)
                s.reset()
                self.assertEqual(json.loads(s.metadata())['contact_model'], expected)


if __name__ == '__main__':
    unittest.main()
