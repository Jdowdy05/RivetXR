"""Transactional user cubes and QDIA, with the pinned real-host Newton runtime."""
import json
import math
import os
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "quest/app/src/main/python"))
from quest_sim.runtime import Session, HEADER, HOME
from quest_sim.scene_objects import parse_object_command
from quest_sim.scene_details import DETAILS_HEADER, DETAILS_OBJECT, DETAILS_CONTACT


def spawn(object_id, pose=None):
    return dict(version=1, op="spawn", id=object_id,
                pose=pose or [.8 + object_id * .1, 0, .7, 0, 0, 0, 1],
                half_extents=[.025] * 3)


class ObjectParserTests(unittest.TestCase):
    def test_valid_commands_are_detached_and_immutable(self):
        value = spawn(1)
        command = parse_object_command(json.dumps(value))
        value["pose"][0] = 8
        self.assertEqual(command.pose[0], .9)
        with self.assertRaises((TypeError, AttributeError)):
            command.pose[0] = 0
        for op in ("remove", "reset"):
            self.assertEqual(parse_object_command(json.dumps(dict(version=1, op=op, id=1))).op, op)

    def test_invalid_inputs_and_native_float_boundaries(self):
        invalid = [[], {}, dict(spawn(1), version=True), dict(spawn(1), version=2),
                   dict(spawn(1), op="attach"), dict(spawn(1), id=0), dict(spawn(1), id=True),
                   dict(spawn(1), id=2**32), dict(spawn(1), extra=0)]
        for field, values in (("pose", [[0]*7, [0,0,0,0,0,0,1.002], [101,0,0,0,0,0,1],
                                        [False,0,0,0,0,0,1], [0,0,math.nan,0,0,0,1], [10**500,0,0,0,0,0,1]]),
                              ("half_extents", [[.025,.03,.025], [.0049]*3, [.101]*3, [math.inf]*3, [.025]*2])):
            invalid.extend(dict(spawn(1), **{field: value}) for value in values)
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_object_command(json.dumps(value))
        for raw in ('{"version":1,"version":1,"op":"remove","id":1}',
                    '{"version":1,"op":"remove","id":1,"id":2}',
                    '{"version":1,"op":"reset","id":1,"pose":[]}'):
            with self.assertRaises(ValueError):
                parse_object_command(raw)
        value = spawn(2**32-1, [0,0,1,0,0,0,1])
        minimum = struct.unpack("<f", struct.pack("<f", .005))[0]
        value["half_extents"] = [minimum] * 3
        self.assertEqual(parse_object_command(json.dumps(value)).half_extents, (minimum,) * 3)
        value["half_extents"] = [math.nextafter(minimum, -math.inf)] * 3
        with self.assertRaises(ValueError):
            parse_object_command(json.dumps(value))
        maximum = struct.unpack("<f", struct.pack("<f", .1))[0]
        value["half_extents"] = [maximum] * 3
        self.assertEqual(parse_object_command(json.dumps(value)).half_extents, (maximum,) * 3)
        value["half_extents"] = [math.nextafter(maximum, math.inf)] * 3
        with self.assertRaises(ValueError):
            parse_object_command(json.dumps(value))


class SceneDetailsTests(unittest.TestCase):
    def test_object_contact_filter_truncation_signed_force_and_unsampled_mode(self):
        import numpy as np
        from quest_sim.scene_details import details_bytes
        from quest_sim.scene_objects import SceneObject
        calls = []
        def force(_model, _data, index, result):
            calls.append(index)
            result[:] = 0
            result[0] = -.25-index
        contacts = SimpleNamespace(geom=np.array([[0,0]]*5+[[0,1]]*35),
                                   pos=np.zeros((40,3)), frame=np.tile([0,0,-1,1,0,0,0,1,0], (40,1)))
        solver = SimpleNamespace(mj_model=SimpleNamespace(geom_bodyid=np.array([0,1])),
                                 mj_data=SimpleNamespace(ncon=40, contact=contacts),
                                 mjc_body_to_newton=SimpleNamespace(numpy=lambda: np.array([[-1,12]])),
                                 _mujoco=SimpleNamespace(mj_contactForce=force))
        session = SimpleNamespace(box_body=None, objects=(SceneObject(5,(0,0,1,0,0,0,1),(.025,)*3),),
                                  object_bodies={5:12}, solver=solver, np=np,
                                  model=SimpleNamespace(body_count=13), generation=2,step_index=7,sim_time=.035,
                                  _contacts_current=True)
        result = details_bytes(session)
        header = DETAILS_HEADER.unpack_from(result)
        self.assertEqual(header[7:], (1,32,35,1))
        self.assertEqual(calls, list(range(5,37)))
        first = DETAILS_CONTACT.unpack_from(result, 48+24)
        self.assertEqual(first, (-1,12,0,0,0,0,0,-1,-5.25))
        self.assertEqual(len(result), 48+24+32*36)
        calls.clear()
        self.assertEqual(DETAILS_HEADER.unpack_from(details_bytes(session,False))[7:], (1,0,0,0))
        self.assertEqual(calls, [])
        session._contacts_current = False
        self.assertEqual(DETAILS_HEADER.unpack_from(details_bytes(session,True))[7:], (1,0,0,2))
        self.assertEqual(calls, [], "stale diagnostics must not read old native contact forces")
        self.assertEqual(DETAILS_HEADER.unpack_from(details_bytes(session,False))[7:], (1,0,0,0))
        session._contacts_current = True
        solver._mujoco.mj_contactForce = lambda _m,_d,_i,f: f.fill(math.nan)
        with self.assertRaises(FloatingPointError):details_bytes(session)


@unittest.skipUnless(os.environ.get("FRANKA_DESCRIPTION_ROOT"), "real Franka assets required")
class SceneObjectTests(unittest.TestCase):
    def make(self, floating=False, cached=True, profile=None):
        return Session(os.environ["FRANKA_DESCRIPTION_ROOT"],
                       json.dumps(dict(floating_base=floating, initial_base_pose=[0,0,.7,0,0,0,1])),
                       cpu_cache=cached, contact_profile=profile)

    def command(self, session, payload):
        return session.command(json.dumps(payload))

    def object_map(self, session):
        blob = session.details_bytes(False)
        header = DETAILS_HEADER.unpack_from(blob)
        self.assertEqual(header[:3], (b"QDIA", 1, 48))
        self.assertEqual(header[3:7], (session.generation, session.model.body_count, session.step_index, session.sim_time))
        self.assertEqual(header[8:], (0, 0, 0))
        self.assertEqual(len(blob), 48 + header[7] * 24)
        return {record[0]: record[1] for record in
                (DETAILS_OBJECT.unpack_from(blob, 48+i*24) for i in range(header[7]))}

    def capture(self, session):
        """Capture integration storage by stable identity, excluding derived contacts."""
        result = {}
        model, solver, data = session.model, session.solver, session.solver.mj_data
        for owner_name in ("state", "next_state"):
            owner = getattr(session, owner_name)
            for index, label in enumerate(model.body_label):
                for name in ("body_q", "body_qd", "body_f"):
                    result[f"{owner_name}.{label}.{name}"] = getattr(owner, name).numpy()[index].copy()
        q_start, qd_start = model.joint_q_start.numpy(), model.joint_qd_start.numpy()
        mjq, mjd = solver.mj_q_start.numpy(), solver.mj_qd_start.numpy()
        for index, label in enumerate(model.joint_label):
            q, qd = slice(q_start[index], q_start[index+1]), slice(qd_start[index], qd_start[index+1])
            for owner_name in ("state", "next_state"):
                owner = getattr(session, owner_name)
                result[f"{owner_name}.{label}.joint_q"] = owner.joint_q.numpy()[q].copy()
                result[f"{owner_name}.{label}.joint_qd"] = owner.joint_qd.numpy()[qd].copy()
            for name, where in (("joint_target_q", q), ("joint_target_qd", qd), ("joint_f", qd)):
                result[f"control.{label}.{name}"] = getattr(session.control, name).numpy()[where].copy()
            result[f"mujoco.{label}.qpos"] = data.qpos[mjq[index]:mjq[index]+q.stop-q.start].copy()
            for name in ("qvel", "qacc_warmstart", "qfrc_applied"):
                result[f"mujoco.{label}.{name}"] = getattr(data, name)[mjd[index]:mjd[index]+qd.stop-qd.start].copy()
        for mj_body, body in enumerate(solver.mjc_body_to_newton.numpy()[0]):
            label = model.body_label[body] if body >= 0 else "world"
            result[f"mujoco.{label}.xfrc_applied"] = data.xfrc_applied[mj_body].copy()
        for name in ("ctrl", "act", "history", "mocap_pos", "mocap_quat", "eq_active"):
            result[f"mujoco.{name}"] = getattr(data, name).copy()
        result["counters"] = (session.step_index, session.sim_time, session.step_cpu_ms, solver._step, data.time)
        return result

    def assert_preserved(self, session, old, excluded=()):
        current = self.capture(session)
        for key, value in old.items():
            if any(f".{label}." in key for label in excluded):
                continue
            self.assertIn(key, current)
            session.np.testing.assert_array_equal(current[key], value, err_msg=key)

    def move_scene(self, session, count=7):
        for step in range(count):
            target = HOME[:7].copy()
            target[0] = .17
            session.step(.005, target, .65)

    def test_fixed_and_floating_survivors_keep_exact_state_when_indices_shift(self):
        for floating in (False, True):
            with self.subTest(floating=floating):
                session = self.make(floating)
                for object_id in (1, 2, 3):
                    self.command(session, spawn(object_id))
                self.move_scene(session)
                session.control.joint_f.numpy()[:] = .013
                session.control.joint_target_qd.numpy()[:] = .017
                old_map, before = self.object_map(session), self.capture(session)
                old_views, old_solver = session._cpu_views, session.solver
                blob = self.command(session, dict(version=1, op="remove", id=1))
                self.assertEqual(HEADER.unpack_from(blob)[3:5], (14, 2))
                self.assert_preserved(session, before, ("cube_1", "cube_1_free_joint"))
                self.assertEqual(self.object_map(session)[2], old_map[2]-1)
                self.assertIsNot(session._cpu_views, old_views)
                self.assertIsNot(session.solver, old_solver)
                before = self.capture(session)
                self.command(session, spawn(4))
                self.assert_preserved(session, before)
                self.move_scene(session, 1)

    def test_move_and_reset_only_selected_object_and_retain_original_home(self):
        session = self.make()
        home = [.9,.1,.8,0,0,0,1]
        self.command(session, spawn(1, home));self.command(session, spawn(2))
        self.move_scene(session)
        for op, pose in (("move", [1.2,-.1,.9,0,0,math.sin(.2),math.cos(.2)]), ("reset", home)):
            before = self.capture(session)
            payload = dict(version=1, op=op, id=1)
            if op == "move":payload["pose"] = pose
            self.command(session, payload)
            self.assert_preserved(session, before, ("cube_1", "cube_1_free_joint"))
            body = self.object_map(session)[1]
            for state in (session.state, session.next_state):
                session.np.testing.assert_allclose(state.body_q.numpy()[body], pose, atol=1e-7)
                session.np.testing.assert_array_equal(state.body_qd.numpy()[body], 0)
                session.np.testing.assert_array_equal(state.body_f.numpy()[body], 0)
        self.command(session, dict(version=1,op="move",id=1,pose=[1.2,0,1,0,0,0,1]))
        session.reset()
        body = self.object_map(session)[1]
        session.np.testing.assert_allclose(session.state.body_q.numpy()[body], home, atol=1e-7,
                                           err_msg="full Reset must retain the original spawn home after a move")

    def test_capacity_id_reuse_invalid_and_transfer_failure_are_transactional(self):
        session = self.make()
        session.command("spawn_box")
        for object_id in range(1, 9):self.command(session, spawn(object_id))
        self.assertEqual(len(self.object_map(session)), 9)
        before = self.capture(session);old_model = session.model;generation = session.generation
        for payload in (spawn(9), spawn(1), dict(version=1, op="remove", id=99), dict(version=1, op="reset", id=0)):
            with self.assertRaises(ValueError):self.command(session, payload)
        self.assertIs(session.model, old_model);self.assertEqual(session.generation, generation)
        self.assert_preserved(session, before)
        self.command(session, dict(version=1, op="remove", id=1))
        with self.assertRaises(ValueError):self.command(session, spawn(1))
        before = self.capture(session);old_model = session.model;generation = session.generation
        with mock.patch("quest_sim.scene_objects.restore_object_state", side_effect=RuntimeError("injected transfer failure")):
            with self.assertRaisesRegex(RuntimeError, "injected transfer failure"):
                self.command(session, spawn(9))
        self.assertIs(session.model, old_model);self.assertEqual(session.generation, generation)
        self.assert_preserved(session, before)
        build = Session._build
        def renamed_joint(candidate):
            build(candidate)
            candidate.model.joint_label[1] = "unexpected_robot_joint_rename"
        with mock.patch.object(Session, "_build", renamed_joint):
            with self.assertRaises(ValueError):self.command(session, spawn(9))
        self.assertIs(session.model, old_model)
        self.assert_preserved(session, before)
        self.command(session, spawn(9))  # A failed spawn must not consume its ID.

    def test_room_change_and_deliberate_global_reset_keep_object_registry(self):
        session = self.make()
        self.command(session, spawn(1));self.command(session, spawn(2))
        self.move_scene(session)
        room = dict(version=1, revision=1, enabled=True, colliders=[
            dict(kind="floor", pose=[0,0,-.025,0,0,0,1], half_extents=[3,3,.025]),
            dict(kind="table", pose=[1,0,.375,0,0,0,1], half_extents=[.4,.4,.025])])
        before = self.capture(session);mapping = self.object_map(session)
        session.set_environment(json.dumps(room));self.assert_preserved(session, before)
        self.assertEqual(self.object_map(session), mapping)
        for _ in range(140):session.step(.005, HOME[:7], 0)
        for body in self.object_map(session).values():
            self.assertAlmostEqual(float(session.state.body_q.numpy()[body,2]), .425, delta=.01)
        session.reset();self.assertEqual(session.step_index, 0);self.assertEqual(len(self.object_map(session)), 2)
        self.assertTrue(session.environment.enabled)

    def test_details_contact_attribution_signed_force_and_packing(self):
        session = self.make()
        self.command(session, spawn(1, [.9,0,.2,0,0,0,1]))
        for _ in range(120):session.step(.005, HOME[:7], 0)
        before = self.capture(session)
        data = session.details_bytes()
        header = DETAILS_HEADER.unpack_from(data)
        self.assertEqual(header[7], 1)
        self.assertGreater(header[8], 0);self.assertGreaterEqual(header[9], header[8])
        self.assertEqual(header[10], 0)
        self.assertEqual(len(data), 48+24*header[7]+36*header[8])
        body = self.object_map(session)[1]
        contacts = [DETAILS_CONTACT.unpack_from(data, 48+24*header[7]+i*36) for i in range(header[8])]
        self.assertTrue(all(body in c[:2] for c in contacts))
        self.assertTrue(any(-1 in c[:2] for c in contacts))
        self.assertTrue(all(all(math.isfinite(v) for v in c[2:]) for c in contacts))
        self.assertGreater(sum(c[-1] for c in contacts), 0)
        self.assert_preserved(session, before)
        with self.assertRaises(ValueError):session.details_bytes(1)

    def test_cached_and_upstream_new_object_operations_match(self):
        sessions = [self.make(cached=value) for value in (False, True)]
        operations = [spawn(1), spawn(2), dict(version=1, op="remove", id=1),
                      dict(version=1, op="move", id=2, pose=[1.1,.2,.4,0,0,0,1]), dict(version=1, op="reset", id=2)]
        for operation in operations:
            for session in sessions:
                self.command(session, operation)
                self.move_scene(session, 2)
            for key, value in self.capture(sessions[0]).items():
                if key == "counters":continue  # CPU wall-clock time differs.
                sessions[0].np.testing.assert_array_equal(self.capture(sessions[1])[key], value, err_msg=key)
            self.assertEqual(sessions[0].details_bytes(), sessions[1].details_bytes())

    def test_real_gripper_produces_force_on_both_cube_surfaces(self):
        for profile in ('legacy_mesh_v1', 'five_pads_v1', 'pad_manipulation_v1'):
            with self.subTest(profile=profile):
                session = self.make(profile=profile)
                poses = session.state.body_q.numpy()
                rotation = poses[9,3:].copy()
                center = .5*(poses[10,:3]+poses[11,:3]) + session.np.asarray(
                    session.wp.quat_rotate(session.wp.quat(*rotation), session.wp.vec3(0,0,.03)))
                self.command(session, spawn(1, [*map(float,center), *map(float,rotation)]))
                for _ in range(20):session.step(.005,HOME[:7],1.)
                blob = session.details_bytes();header = DETAILS_HEADER.unpack_from(blob)
                contacts = [DETAILS_CONTACT.unpack_from(blob,48+header[7]*24+i*36) for i in range(header[8])]
                body = self.object_map(session)[1]
                for finger in (10,11):
                    self.assertTrue(any(finger in c[:2] and body in c[:2] and c[-1] > 0 for c in contacts))
                if profile == 'pad_manipulation_v1':
                    self.assertGreater(header[9], header[8])
                    self.assertEqual(header[10], 1)  # Both fingers represented even when truncated.
        # This is contact proof only: the unsupported cube need not remain held.

    def test_base_edit_retires_contacts_until_existing_physics_refresh(self):
        session = self.make()
        poses = session.state.body_q.numpy()
        rotation = poses[9,3:].copy()
        center = .5*(poses[10,:3]+poses[11,:3]) + session.np.asarray(
            session.wp.quat_rotate(session.wp.quat(*rotation), session.wp.vec3(0,0,.03)))
        self.command(session, spawn(1, [*map(float,center), *map(float,rotation)]))
        for _ in range(20):session.step(.005,HOME[:7],1.)
        self.assertGreater(DETAILS_HEADER.unpack_from(session.details_bytes())[8], 0)
        step = session.step_index
        session.set_base_pose([0,0,1.7,0,0,0,1])
        before = self.capture(session)
        header = DETAILS_HEADER.unpack_from(session.details_bytes(True))
        self.assertEqual(header[7:], (1,0,0,2), "moved-base contacts must be marked not current, not sampled")
        self.assertEqual(session.step_index, step)
        self.assert_preserved(session, before)
        self.assertEqual(DETAILS_HEADER.unpack_from(session.details_bytes(False))[7:], (1,0,0,0),
                         "mapping-only response keeps its existing omission contract")
        session.step(.005,HOME[:7],1.)
        self.assertEqual(DETAILS_HEADER.unpack_from(session.details_bytes(True))[10] & 2, 0)
        session.set_base_pose([0,0,1.6,0,0,0,1]);session.reset()
        self.assertEqual(DETAILS_HEADER.unpack_from(session.details_bytes(True))[10] & 2, 0,
                         "existing reset mj_forward restores current contacts")
        session.set_base_pose([0,0,1.5,0,0,0,1])
        session.set_environment(json.dumps(dict(version=1,revision=1,enabled=False,colliders=[])))
        self.assertEqual(DETAILS_HEADER.unpack_from(session.details_bytes(True))[10] & 2, 0,
                         "state-preserving room rebuild/forward restores current contacts")
        session.set_base_pose([0,0,1.4,0,0,0,1])
        self.command(session, spawn(2))
        self.assertEqual(DETAILS_HEADER.unpack_from(session.details_bytes(True))[10] & 2, 0,
                         "state-preserving object rebuild/forward restores current contacts")


if __name__ == "__main__":unittest.main()
