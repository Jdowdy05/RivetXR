"""Bounded, simulation-only scene acceptance on the caller's active CPU runtime.

No room scans, persisted settings, robot/controller input or network are used.
This checks numerical contacts and edit invariants, not grasping, alignment,
physical-room accuracy, frame rate or cross-platform numerical equivalence.
"""
from __future__ import annotations

import json
import math
import platform
import sys
import time
import traceback


DT = .005


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _capture(session):
    """Independent semantic integration capture keyed by stable body/joint label.

    Derived contact storage is intentionally excluded: a geometry edit must
    regenerate contacts, while integration state and survivor inputs survive.
    """
    result = {}
    model, solver, data = session.model, session.solver, session.solver.mj_data
    for owner_name in ('state', 'next_state'):
        owner = getattr(session, owner_name)
        for index, label in enumerate(model.body_label):
            for name in ('body_q', 'body_qd', 'body_f'):
                result[(owner_name, label, name)] = getattr(owner, name).numpy()[index].copy()
    q_start, d_start = model.joint_q_start.numpy(), model.joint_qd_start.numpy()
    mq, md = solver.mj_q_start.numpy(), solver.mj_qd_start.numpy()
    for index, label in enumerate(model.joint_label):
        q = slice(int(q_start[index]), int(q_start[index+1]))
        d = slice(int(d_start[index]), int(d_start[index+1]))
        for owner_name in ('state', 'next_state'):
            owner = getattr(session, owner_name)
            result[(owner_name, label, 'joint_q')] = owner.joint_q.numpy()[q].copy()
            result[(owner_name, label, 'joint_qd')] = owner.joint_qd.numpy()[d].copy()
        for name, where in (('joint_target_q', q), ('joint_target_qd', d), ('joint_f', d)):
            result[('control', label, name)] = getattr(session.control, name).numpy()[where].copy()
        result[('mujoco', label, 'qpos')] = data.qpos[int(mq[index]):int(mq[index])+q.stop-q.start].copy()
        for name in ('qvel', 'qacc_warmstart', 'qfrc_applied'):
            result[('mujoco', label, name)] = getattr(data, name)[int(md[index]):int(md[index])+d.stop-d.start].copy()
    for index, body in enumerate(solver.mjc_body_to_newton.numpy()[0]):
        label = model.body_label[int(body)] if body >= 0 else '@world'
        result[('mujoco', label, 'xfrc_applied')] = data.xfrc_applied[index].copy()
    for name in ('ctrl', 'act', 'history', 'mocap_pos', 'mocap_quat', 'eq_active'):
        result[('mujoco', '@global', name)] = getattr(data, name).copy()
    result[('session', '@global', 'counters')] = (session.step_index, session.sim_time,
        session.step_cpu_ms, solver._step, float(data.time))
    result[('session', '@global', 'settings')] = dict(session.settings)
    return result


def _preserved(session, before, excluded=()):
    current = _capture(session)
    comparisons = 0
    for key, previous in before.items():
        if key[1] in excluded:
            continue
        _require(key in current, f'Survivor missing: {key}')
        value = current[key]
        if hasattr(previous, 'dtype'):
            _require(previous.dtype == value.dtype and previous.shape == value.shape
                     and previous.tobytes() == value.tobytes(), f'Survivor state changed: {key}')
        else:
            _require(previous == value, f'Session state changed: {key}')
        comparisons += 1
    return comparisons


def _details(session, sample=True):
    from .scene_details import DETAILS_HEADER, DETAILS_OBJECT, DETAILS_CONTACT
    raw = session.details_bytes(sample)
    header = DETAILS_HEADER.unpack_from(raw)
    _require(header[:3] == (b'QDIA', 1, 48), 'QDIA framing changed')
    _require(header[3:7] == (session.generation, session.model.body_count,
                            session.step_index, session.sim_time), 'QDIA state identity mismatch')
    objects, contacts = header[7:9]
    _require(len(raw) == 48+24*objects+36*contacts, 'QDIA length mismatch')
    mapping = {value[0]: value[1] for value in
               (DETAILS_OBJECT.unpack_from(raw, 48+i*24) for i in range(objects))}
    _require(len(mapping) == objects, 'QDIA duplicated object identity')
    records = [DETAILS_CONTACT.unpack_from(raw, 48+objects*24+i*36) for i in range(contacts)]
    _require(all(all(math.isfinite(x) for x in record[2:]) for record in records),
             'Nonfinite contact diagnostic')
    return header, mapping, records


def _room(revision, enabled=True):
    return dict(version=1, revision=revision, enabled=enabled, colliders=[
        dict(kind='floor', pose=[0, 0, -.025, 0, 0, 0, 1], half_extents=[3, 3, .025]),
        dict(kind='table', pose=[1, 0, .375, 0, 0, 0, 1], half_extents=[.3, .3, .025]),
    ] if enabled else [])


def run(asset_root):
    """Bridge entry returning strict JSON, including partial evidence on failure."""
    proof = dict(schema_version=1, passed=False, physics_dt=DT, checks=[], step_count=0,
                 scope='Synthetic scene numerical/edit acceptance on this process; no grasp, room-scan, visual or performance claim',
                 platform=sys.platform, machine=platform.machine(), python=sys.version,
                 field_comparisons=0)
    started = time.monotonic()
    active_check = 'create_session'
    try:
        from .runtime import Session, HOME
        session = Session(str(asset_root), json.dumps(dict(physics_dt=DT,
                          initial_base_pose=[0, 0, 1.2, 0, 0, 0, 1])))
        proof['metadata'] = json.loads(session.metadata())
        _require(proof['metadata']['backend'] == 'Newton SolverMuJoCo CPU', 'Unexpected backend')
        _require(session.cpu_cache and session.solver.use_mujoco_cpu, 'Expected active cached CPU path')
        proof['state_dtype'] = str(session.state.joint_q.numpy().dtype)
        proof['mujoco_dtype'] = str(session.solver.mj_data.qpos.dtype)

        def passed(name, **evidence):
            proof['checks'].append(dict(name=name, passed=True, **evidence))

        def preserve(before, excluded=()):
            proof['field_comparisons'] += _preserved(session, before, excluded)

        def command(op, object_id, pose=None):
            payload = dict(version=1, op=op, id=object_id)
            if pose is not None:
                payload['pose'] = pose
            if op == 'spawn':
                payload['half_extents'] = [.025]*3
            return session.command(json.dumps(payload))

        def steps(count, motion=False):
            targets = HOME[:7].copy()
            if motion:
                targets[0] = .17
            for _ in range(count):
                session.step(DT, targets, .65 if motion else 0.)
                proof['step_count'] += 1

        def reject(name, operation):
            before, model, solver = _capture(session), session.model, session.solver
            generation, environment = session.generation, session.environment
            objects, used_ids = session.objects, session.used_object_ids
            try:
                operation()
            except ValueError:
                pass
            else:
                raise AssertionError('Invalid transaction accepted: '+name)
            _require(session.model is model and session.solver is solver
                     and session.generation == generation and session.environment is environment
                     and session.objects == objects and session.used_object_ids == used_ids,
                     'Invalid transaction modified scene identity: '+name)
            preserve(before)
            passed(name)

        active_check = 'spawn_preserves_active_robot'
        steps(9, motion=True)
        session.control.joint_f.numpy()[:] = .013
        session.control.joint_target_qd.numpy()[:] = .017
        before = _capture(session)
        home = [1., 0., .7, 0, 0, 0, 1]
        command('spawn', 1, home)
        preserve(before)
        before = _capture(session)
        command('spawn', 2, [1.7, 0, .7, 0, 0, 0, 1])
        preserve(before)
        passed(active_check, stable_ids=sorted(_details(session, False)[1]))

        active_check = 'move_reset_selected_cube_only'
        steps(7, motion=True)
        for op, pose in (('move', [1.1, .1, .9, 0, 0, math.sin(.2), math.cos(.2)]), ('reset', home)):
            before = _capture(session)
            command(op, 1, pose if op == 'move' else None)
            preserve(before, ('cube_1', 'cube_1_free_joint'))
            body = _details(session, False)[1][1]
            for state in (session.state, session.next_state):
                _require(session.np.allclose(state.body_q.numpy()[body], pose, rtol=0, atol=1e-7), 'Selected cube pose incorrect')
                _require(not session.np.any(state.body_qd.numpy()[body]) and not session.np.any(state.body_f.numpy()[body]),
                         'Selected cube retained velocity/force')
        passed(active_check)

        active_check = 'delete_preserves_survivor_across_index_shift'
        before, old_mapping = _capture(session), _details(session, False)[1]
        command('remove', 1)
        preserve(before, ('cube_1', 'cube_1_free_joint'))
        mapping = _details(session, False)[1]
        _require(1 not in mapping and mapping[2] == old_mapping[2]-1, 'Stable ID did not survive index shift')
        passed(active_check, surviving_id=2, old_body=old_mapping[2], new_body=mapping[2])
        active_check = 'retired_object_id_rejected'
        reject(active_check, lambda: command('spawn', 1, home))
        active_check = 'unknown_object_rejected'
        reject(active_check, lambda: command('remove', 99))
        before = _capture(session)
        command('spawn', 3, home)
        preserve(before)
        passed('fresh_object_id_after_delete', stable_ids=sorted(_details(session, False)[1]))

        for revision, enabled in ((1, True), (2, False), (3, True)):
            active_check = f'room_revision_{revision}_preserves_scene'
            before, mapping = _capture(session), _details(session, False)[1]
            session.set_environment(json.dumps(_room(revision, enabled)))
            preserve(before)
            _require(_details(session, False)[1] == mapping, 'Room edit changed stable identity mapping')
            meta = json.loads(session.metadata())['environment']
            _require(meta['revision'] == revision and meta['enabled'] == enabled
                     and meta['collider_count'] == (2 if enabled else 0), 'Wrong applied room metadata')
            passed(active_check, collider_count=meta['collider_count'])

        bad = _room(4)
        bad['colliders'][1]['half_extents'][2] = -1
        active_check = 'invalid_room_batch_rolls_back'
        reject(active_check, lambda: session.set_environment(json.dumps(bad)))
        active_check = 'stale_room_revision_rolls_back'
        reject(active_check, lambda: session.set_environment(json.dumps(_room(2))))

        active_check = 'bounded_table_and_floor_contact'
        # Zero only the diagnostic inputs seeded above through their public arrays;
        # do not change solver accuracy, contact settings, state or scene geometry.
        session.control.joint_f.zero_()
        session.control.joint_target_qd.zero_()
        steps(220)
        header, mapping, contacts = _details(session)
        table_body, floor_body = mapping[3], mapping[2]
        table_z, floor_z = (float(session.state.body_q.numpy()[body, 2]) for body in (table_body, floor_body))
        _require(.415 < table_z < .435, f'Table cube not resting at 0.425 m: {table_z}')
        _require(.015 < floor_z < .035, f'Cube outside table did not reach floor: {floor_z}')
        for body in (table_body, floor_body):
            _require(any(body in record[:2] and -1 in record[:2] and record[-1] > 0 for record in contacts),
                     'Missing positive cube/static-surface normal contact')
            _require(float(session.np.linalg.norm(session.state.body_qd.numpy()[body])) < .05,
                     'Cube did not settle')
        passed(active_check, table_center_z_m=table_z, floor_center_z_m=floor_z,
               expected_table_center_z_m=.425, expected_floor_center_z_m=.025,
               tolerance_m=.01, sampled_contacts=header[8], total_contacts=header[9], flags=header[10])

        active_check = 'qdia_mapping_only_has_no_samples_or_state_change'
        before = _capture(session)
        header, _, records = _details(session, False)
        _require(header[8:] == (0, 0, 0) and not records, 'Mapping-only QDIA sampled contacts')
        preserve(before)
        passed(active_check)

        active_check = 'qdia_base_change_retires_contacts'
        room_shapes = [i for i, name in enumerate(session.model.shape_label) if name.startswith('room_')]
        geometry = session.model.shape_transform.numpy()[room_shapes].copy()
        cube_poses = session.state.body_q.numpy()[[table_body, floor_body]].copy()
        session.set_base_pose([.1, 0, 1.3, 0, 0, 0, 1])
        _require(session.np.array_equal(geometry, session.model.shape_transform.numpy()[room_shapes])
                 and session.np.array_equal(cube_poses, session.state.body_q.numpy()[[table_body, floor_body]]),
                 'Base move changed world room/object poses')
        before = _capture(session)
        header, _, records = _details(session)
        _require(header[8:] == (0, 0, 2) and not records, 'Base edit returned current contact samples')
        preserve(before)
        passed(active_check, unavailable_flag=header[10])
        active_check = 'qdia_step_restores_current_contacts'
        steps(1)
        header, _, contacts = _details(session)
        _require(header[10] & 2 == 0 and header[8] > 0, 'Physics step did not restore current contacts')
        passed(active_check, sampled_contacts=header[8])

        active_check = 'table_removal_preserves_state_then_cube_falls_to_ground'
        before = _capture(session)
        session.set_environment(json.dumps(_room(4, False)))
        preserve(before)
        _require(not any(table_body in record[:2] for record in _details(session)[2]),
                 'Removed table contact survived geometry replacement')
        steps(160)
        z = float(session.state.body_q.numpy()[table_body, 2])
        _require(.015 < z < .035, f'Released table cube did not fall to ground: {z}')
        _require(any(table_body in record[:2] and -1 in record[:2] and record[-1] > 0
                     for record in _details(session)[2]), 'Released table cube lacks ground contact')
        passed(active_check, center_z_m=z, expected_center_z_m=.025, tolerance_m=.01)

        active_check = 'room_reapply_then_full_reset_keeps_registry_and_homes'
        before = _capture(session)
        session.set_environment(json.dumps(_room(5)))
        preserve(before)
        session.reset()
        _require(session.step_index == 0 and session.sim_time == 0 and session.environment.revision == 5,
                 'Global reset did not retain room or reset time')
        mapping = _details(session, False)[1]
        _require(sorted(mapping) == [2, 3], 'Global reset lost stable cube registry')
        for obj in session.objects:
            _require(session.np.allclose(session.state.body_q.numpy()[mapping[obj.id]], obj.home_pose,
                                        rtol=0, atol=1e-7), 'Global reset lost original cube home')
        steps(120)
        _require(.415 < float(session.state.body_q.numpy()[mapping[3], 2]) < .435,
                 'Reapplied room table no longer supports reset cube')
        passed(active_check, stable_ids=sorted(mapping), room_revision=session.environment.revision)
        proof['final_metadata'] = json.loads(session.metadata())
        proof['passed'] = True
    except Exception as exception:
        proof['error'] = dict(check=active_check, type=type(exception).__name__,
                             message=str(exception), traceback=traceback.format_exc())
    proof['validation_elapsed_seconds'] = time.monotonic()-started
    # Metadata contains only finite runtime values; reject rather than emit NaN
    # tokens if that contract ever changes.
    return json.dumps(proof, allow_nan=False)
