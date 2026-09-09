"""Room payload and actual Newton CPU scene preservation/contact checks."""
import json
import math
import os
from pathlib import Path
import struct
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "quest/app/src/main/python"))
from quest_sim import room_environment
from quest_sim.runtime import Session, HEADER, HOME


def room(revision=1, *, table=True):
    colliders = [dict(kind="floor", pose=[0., 0., -.025, 0., 0., 0., 1.],
                      half_extents=[3., 3., .025])]
    if table:
        colliders.append(dict(kind="table", pose=[.8, 0., .375, 0., 0., 0., 1.],
                              half_extents=[.3, .3, .025]))
    return dict(version=1, revision=revision, enabled=True, colliders=colliders)


def disabled(revision):
    return dict(version=1, revision=revision, enabled=False, colliders=[])


class RoomPayloadTests(unittest.TestCase):
    def test_complete_batch_is_detached_and_immutable(self):
        payload = room()
        result = room_environment.parse_environment(json.dumps(payload))
        payload["colliders"][0]["pose"][2] = 8.
        self.assertEqual(result.colliders[0].pose[2], -.025)
        with self.assertRaises((AttributeError, TypeError)):
            result.colliders[0].pose[2] = 1.
        self.assertEqual(result.counts(), dict(floor=1, wall=0, table=1))
        self.assertFalse(room_environment.parse_environment(json.dumps(disabled(2))).enabled)

    def test_invalid_batches_are_rejected(self):
        invalid = [[], {}, dict(room(), version=True), dict(room(), revision=-1),
                   dict(room(), revision=2**64), dict(room(), revision=True),
                   dict(room(), enabled=1), dict(room(), colliders=[]),
                   dict(room(), colliders=room()["colliders"] * 33),
                   dict(room(), enabled=False), dict(room(), unexpected=1)]
        for field, value in (("kind", "chair"), ("pose", [0] * 7),
                             ("pose", [101., 0, 0, 0, 0, 0, 1]),
                             ("pose", [0, 0, float("nan"), 0, 0, 0, 1]),
                             ("pose", [False, 0, 0, 0, 0, 0, 1]),
                             ("pose", [0, 0, 0, 0, 0, 0, 1.002]),
                             ("half_extents", [.0049, .1, .1]),
                             ("half_extents", [.1, 20.001, .1]),
                             ("half_extents", [.1, .1, float("inf")])):
            candidate = room()
            candidate["colliders"][0][field] = value
            invalid.append(candidate)
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                room_environment.parse_environment(json.dumps(value))

    def test_inclusive_limits_and_large_numbers(self):
        payload = room()
        payload["revision"] = 2**64 - 1
        payload["colliders"][0]["pose"][:3] = [-100, 100, 0]
        payload["colliders"][0]["half_extents"] = [.005, 20, .005]
        payload["colliders"] += [dict(kind="wall", pose=[0, 1, 1, 0, 0, 0, 1],
                                      half_extents=[1, .01, 1])] * 62
        self.assertEqual(len(room_environment.parse_environment(json.dumps(payload)).colliders), 64)
        payload["colliders"][0]["pose"][0] = 10**500
        with self.assertRaises(ValueError):
            room_environment.parse_environment(json.dumps(payload))

    def test_native_float32_minimum_is_accepted_without_relaxing_the_limit(self):
        native_minimum = struct.unpack("<f", struct.pack("<f", .005))[0]
        payload = room()
        payload["colliders"][0]["half_extents"][2] = native_minimum
        parsed = room_environment.parse_environment(json.dumps(payload))
        self.assertEqual(parsed.colliders[0].half_extents[2], native_minimum)
        payload["colliders"][0]["half_extents"][2] = math.nextafter(native_minimum, -math.inf)
        with self.assertRaises(ValueError):
            room_environment.parse_environment(json.dumps(payload))

    def test_enabled_revision_zero_is_rejected_but_initial_disabled_is_valid(self):
        with self.assertRaises(ValueError):
            room_environment.parse_environment(json.dumps(room(0)))
        self.assertEqual(room_environment.parse_environment(json.dumps(disabled(0))).revision, 0)

    def test_duplicate_keys_are_rejected_at_every_object_level(self):
        raw = json.dumps(room())
        duplicates = (raw.replace('"revision": 1', '"revision": 1, "revision": 2'),
                      raw.replace('"version": 1', '"version": 1, "version": 1'),
                      raw.replace('"kind": "floor"', '"kind": "wall", "kind": "floor"'))
        for payload in duplicates:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                room_environment.parse_environment(payload)


@unittest.skipUnless(os.environ.get("FRANKA_DESCRIPTION_ROOT"), "real Franka assets required")
class RoomSceneTests(unittest.TestCase):
    def make_session(self, **settings):
        return Session(os.environ["FRANKA_DESCRIPTION_ROOT"], json.dumps(settings))

    def integration(self, session):
        mj = session.solver._mujoco
        signature = mj.mjtState.mjSTATE_INTEGRATION
        result = session.np.empty(mj.mj_stateSize(session.solver.mj_model, signature))
        mj.mj_getState(session.solver.mj_model, session.solver.mj_data, result, signature)
        return result

    def box_contact_count(self, session):
        body_ids = session.solver.mj_model.geom_bodyid[session.solver.mj_data.contact.geom]
        newton_bodies = session.solver.mjc_body_to_newton.numpy()[0][body_ids]
        return int(session.np.count_nonzero(session.np.any(newton_bodies == session.box_body, axis=1)))

    def capture(self, session):
        arrays = {}
        for owner in ("state", "next_state", "control"):
            for key, value in vars(getattr(session, owner)).items():
                if isinstance(value, session.wp.array):
                    arrays[f"{owner}.{key}"] = value.numpy().copy()
        return dict(integration=self.integration(session), arrays=arrays,
                    step_index=session.step_index, sim_time=session.sim_time,
                    step_cpu_ms=session.step_cpu_ms, solver_step=session.solver._step)

    def assert_preserved(self, session, before):
        session.np.testing.assert_array_equal(self.integration(session), before["integration"])
        for name, array in before["arrays"].items():
            owner, field = name.split(".")
            session.np.testing.assert_array_equal(getattr(getattr(session, owner), field).numpy(), array)
        for name in ("step_index", "sim_time", "step_cpu_ms"):
            self.assertEqual(getattr(session, name), before[name])
        self.assertEqual(session.solver._step, before["solver_step"])

    def test_toggle_refresh_preserves_live_fixed_and_floating_scene(self):
        for floating in (False, True):
            with self.subTest(floating=floating):
                s = self.make_session(floating_base=floating)
                s.command("spawn_box")
                if not floating:
                    s.set_base_pose([.1, .2, .4, 0, 0, 0, 1])
                for step in range(9):
                    targets = HOME[:7].copy()
                    targets[0] = .2
                    s.step(.005, targets, .7)
                s.control.joint_target_qd.numpy()[:] = .05
                s.control.joint_f.numpy()[:] = .03
                body_names = list(s.model.body_label)
                for payload in (room(1), room(2, table=False), disabled(3)):
                    before = self.capture(s)
                    old_views, old_solver = s._cpu_views, s.solver
                    blob = s.set_environment(json.dumps(payload))
                    self.assert_preserved(s, before)
                    self.assertIsNot(s._cpu_views, old_views)
                    self.assertIsNot(s.solver, old_solver)
                    self.assertEqual(s.model.body_label, body_names)
                    self.assertEqual(HEADER.unpack_from(blob)[3:5], (13, 1))
                    meta = json.loads(s.metadata())["environment"]
                    self.assertEqual(meta["revision"], payload["revision"])
                    self.assertEqual(meta["enabled"], payload["enabled"])
                    self.assertEqual(meta["collider_count"], len(payload["colliders"]))
                    s.step(.005, targets, .7)

    def test_invalid_and_failed_rebuild_leave_previous_scene_intact(self):
        s = self.make_session()
        s.step(.005, HOME[:7], .4)
        s.set_environment(json.dumps(room(1)))
        original_model, original_solver = s.model, s.solver
        original_environment = s.environment
        original_generation = s.generation
        before = self.capture(s)
        bad = room(2)
        bad["colliders"][-1]["half_extents"][-1] = -1
        for payload in (bad, disabled(0), room(1, table=False)):
            with self.assertRaises(ValueError):
                s.set_environment(json.dumps(payload))
        with mock.patch.object(Session, "_restore_environment_state", side_effect=RuntimeError("injected transfer failure")):
            with self.assertRaisesRegex(RuntimeError, "injected transfer failure"):
                s.set_environment(json.dumps(room(2)))
        self.assertIs(s.model, original_model)
        self.assertIs(s.solver, original_solver)
        self.assertIs(s.environment, original_environment)
        self.assertEqual(s.generation, original_generation)
        self.assert_preserved(s, before)
        self.assertEqual(s.set_environment(json.dumps(room(1))), s.snapshot())

    def test_box_contacts_bounded_table_and_room_stays_at_world_pose(self):
        s = self.make_session()
        s.command("spawn_box")
        s.set_environment(json.dumps(room()))
        room_shapes = [i for i, label in enumerate(s.model.shape_label) if label.startswith("room_")]
        self.assertEqual(len(room_shapes), 2)
        s.np.testing.assert_array_equal(s.model.shape_body.numpy()[room_shapes], [-1, -1])
        geometry = s.model.shape_transform.numpy()[room_shapes].copy()
        s.set_base_pose([0, 0, 1.2, 0, 0, 0, 1])
        s.np.testing.assert_array_equal(s.model.shape_transform.numpy()[room_shapes], geometry)
        contacts = 0
        for _ in range(180):
            s.step(.005, HOME[:7], 0.)
            contacts = max(contacts, self.box_contact_count(s))
        z = float(s.state.body_q.numpy()[s.box_body, 2])
        self.assertGreater(z, .435)
        self.assertLess(z, .465)
        self.assertGreater(contacts, 0)
        before = self.capture(s)
        s.set_environment(json.dumps(disabled(2)))
        self.assert_preserved(s, before)
        self.assertEqual(self.box_contact_count(s), 0, "old box/table contacts survived geometry removal")
        for _ in range(120):
            s.step(.005, HOME[:7], 0.)
        self.assertLess(float(s.state.body_q.numpy()[s.box_body, 2]), .075)

    def test_table_has_finite_bounds(self):
        s = self.make_session(initial_base_pose=[0, 0, 1.2, 0, 0, 0, 1])
        s.command("spawn_box")
        payload = room()
        payload["colliders"][1]["pose"][0] = 1.4
        s.set_environment(json.dumps(payload))
        for _ in range(180):
            s.step(.005, HOME[:7], 0.)
        z = float(s.state.body_q.numpy()[s.box_body, 2])
        self.assertGreater(z, .035)
        self.assertLess(z, .075, "box outside the table bounds did not reach the floor")

    def test_explicit_rebuilds_retain_environment_and_reset_semantics(self):
        s = self.make_session()
        s.set_environment(json.dumps(room()))
        environment = s.environment
        for operation in (lambda: s.command("spawn_box"),
                          lambda: s.configure('{"gravity_scale":0.5}'),
                          lambda: s.configure('{"floating_base":true}'),
                          lambda: s.reset()):
            s.step(.005, HOME[:7], .3)
            operation()
            self.assertIs(s.environment, environment)
            self.assertEqual(s.step_index, 0)
            self.assertEqual(s.sim_time, 0.)

    def test_default_off_short_trajectory_matches_unchanged_ground(self):
        baseline, toggled = self.make_session(), self.make_session()
        toggled.set_environment(json.dumps(disabled(1)))
        for step in range(32):
            targets = HOME[:7].copy()
            targets[0] = .15
            for s in (baseline, toggled):
                s.step(.005, targets, .6)
            baseline.np.testing.assert_array_equal(self.integration(baseline), self.integration(toggled))
            baseline.np.testing.assert_array_equal(baseline.state.body_q.numpy(), toggled.state.body_q.numpy())
            self.assertEqual(baseline.solver.mj_data.ncon, toggled.solver.mj_data.ncon)


if __name__ == "__main__":
    unittest.main()
