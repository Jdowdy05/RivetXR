"""Opt-in fixed-base gripper proof using the active Session and native contacts.

Synthetic supported pinches only. No lift, attachment, state writes, changed
gains, collision assets, contact settings or solver settings. Off-mode results
are diagnostic context, never a required failure or an acceptance gate.
"""
from __future__ import annotations

import hashlib
import json
import math
import platform
import sys
import time
import traceback


DT = .005
SPEED = .05
FORCE = 5.
TARGET = [0., -math.pi/4, 0., -3*math.pi/4, 0., math.pi/2, math.pi/4]


def _require(value, message):
    if not value:
        raise AssertionError(message)


def _targets(session):
    return session.control.joint_target_q.numpy()[session.arm_q_indices[-2:]].copy()


def _physics_signature(session):
    """Record immutable physics arrays and equality count, excluding targets."""
    m = session.solver.mj_model
    fields = ('geom_solref', 'geom_solimp', 'geom_friction', 'geom_condim', 'geom_contype',
              'geom_conaffinity', 'jnt_actfrcrange', 'actuator_gainprm', 'actuator_biasprm',
              'eq_type', 'eq_obj1id', 'eq_obj2id', 'body_mass', 'body_inertia',
              'geom_type', 'geom_size', 'geom_pos', 'geom_quat')
    digest = hashlib.sha256()
    for name in fields:
        array = getattr(m, name)
        digest.update(name.encode()); digest.update(array.dtype.str.encode()); digest.update(array.tobytes())
    digest.update(repr((m.opt.timestep, m.opt.integrator, m.opt.iterations,
                        m.opt.ls_iterations, m.opt.enableflags, m.opt.disableflags,
                        m.opt.cone, m.opt.impratio)).encode())
    return dict(sha256=digest.hexdigest(), native_equalities=int(m.neq),
                integrator=int(m.opt.integrator), iterations=int(m.opt.iterations),
                line_search_iterations=int(m.opt.ls_iterations))


def _sample(session, initial, stage):
    """Unclipped native signed distances/forces from the last completed solve."""
    m, d, np, mj = session.solver.mj_model, session.solver.mj_data, session.np, session.solver._mujoco
    mapping = session.solver.mjc_body_to_newton.numpy()[0]
    from .scene_details import _normal_force_reader, DETAILS_HEADER, DETAILS_CONTACT
    read_normal = _normal_force_reader(session.solver)
    _require(read_normal is not None, 'Native normal-load diagnostic reader unavailable')
    cube = session.object_bodies[1]
    finger_names = ('panda_finger_joint1', 'panda_finger_joint2')
    joints = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, name) for name in finger_names]
    native_fingers = [int(m.jnt_bodyid[index]) for index in joints]
    records, loads, normal_sums, counts = [], [0., 0.], [0., 0.], [0, 0]
    static_force, peak_torsion = 0., 0.
    dimensions = set()
    force = np.empty(6)
    for index in range(d.ncon):
        if d.contact.exclude[index] != 0 or d.contact.efc_address[index] < 0:
            continue
        native_a, native_b = map(int, m.geom_bodyid[d.contact.geom[index]])
        a, b = int(mapping[native_a]), int(mapping[native_b])
        if cube not in (a, b):
            continue
        mj.mj_contactForce(m, d, index, force)
        normal = float(force[0]); distance = float(d.contact.dist[index])
        _require(all(math.isfinite(float(value)) for value in force) and math.isfinite(distance), 'Nonfinite native contact')
        _require(read_normal(index) == normal, 'Diagnostic normal-load reader disagrees with native contact force')
        if a < 0 or b < 0:
            static_force += normal
        for finger, native_body in enumerate(native_fingers):
            sign = -1. if native_body == native_a else 1. if native_body == native_b else 0.
            if not sign:
                continue
            projection = float(np.dot(d.contact.frame[index, :3]*sign, d.xaxis[joints[finger]]))
            _require(math.isfinite(projection), 'Nonfinite closing-axis projection')
            loads[finger] += max(0., projection*max(0., normal))
            normal_sums[finger] += normal
            counts[finger] += int(normal > 0)
            records.append([finger, distance, normal])
            if normal > 0:
                dimensions.add(int(d.contact.dim[index]))
                peak_torsion = max(peak_torsion, abs(float(force[3])))
    position = session.state.body_q.numpy()[cube, :3].astype(float)
    delta = position-np.asarray(initial[:3])
    qdia = None
    if session.step_index % 20 == 0:
        blob = session.details_bytes()
        header = DETAILS_HEADER.unpack_from(blob)
        _require(header[5] == session.step_index and not header[10] & 2, 'QDIA does not describe current solve')
        sampled = [DETAILS_CONTACT.unpack_from(blob, 48+header[7]*24+i*36) for i in range(header[8])]
        represented = [any(int(mapping[body]) in c[:2] and cube in c[:2] and c[-1] > 0 for c in sampled)
                       for body in native_fingers]
        for slot in (0, 1):
            if normal_sums[slot] > 1e-6:
                _require(represented[slot], 'Loaded finger omitted from sampled contact diagnostics')
        qdia = dict(sampled=header[8], total=header[9], flags=header[10], represented_fingers=represented)
    return dict(step=session.step_index, stage=stage, sim_time=session.sim_time,
                target_q=_targets(session).tolist(), measured_q=session.joint_positions()[-2:],
                cube_xyz=position.tolist(), cube_displacement_m=float(np.linalg.norm(delta)),
                cube_horizontal_displacement_m=float(np.linalg.norm(delta[:2])),
                opposing_force_n=loads, normal_force_n=normal_sums, positive_contacts=counts,
                static_normal_force_n=static_force, active_finger_contacts=records,
                contact_dimensions=sorted(dimensions), peak_torsional_torque_abs_nm=peak_torsion,
                qdia=qdia,
                control=json.loads(session.metadata())['gripper_control'])


def _summary(rows):
    contacts = [c for row in rows for c in row['active_finger_contacts']]
    loaded = [c for c in contacts if c[2] > 0]
    return dict(samples=len(rows), bilateral_samples=sum(all(c > 0 for c in row['positive_contacts']) for row in rows),
                max_cube_displacement_m=max(row['cube_displacement_m'] for row in rows),
                max_horizontal_displacement_m=max(row['cube_horizontal_displacement_m'] for row in rows),
                minimum_active_finger_distance_m=min((c[1] for c in contacts), default=None),
                minimum_force_bearing_finger_distance_m=min((c[1] for c in loaded), default=None),
                peak_opposing_force_n=max(max(row['opposing_force_n']) for row in rows),
                mean_max_opposing_force_n=sum(max(row['opposing_force_n']) for row in rows)/len(rows),
                contact_dimensions=sorted({dim for row in rows for dim in row['contact_dimensions']}),
                max_finger_point_counts=[max(row['positive_contacts'][i] for row in rows) for i in (0,1)],
                peak_torsional_torque_abs_nm=max(row['peak_torsional_torque_abs_nm'] for row in rows),
                qdia_samples=sum(row['qdia'] is not None for row in rows),
                final=rows[-1])


def run(asset_root, contact_profile=None):
    """Return bounded strict JSON with partial evidence on failure."""
    proof = dict(schema_version=1, passed=False, physics_dt=DT, speed_mps=SPEED,
                 desired_force_n=FORCE, step_count=0, checks=[], platform=sys.platform,
                 machine=platform.machine(), python=sys.version,
                 scope='Fixed-base supported synthetic pinch and release; no lift, visual, real-room or performance claim',
                 contact_policy='Raw active native contacts exclude==0 and efc_address>=0; negative distance is penetration; no additional forward solve')
    started = time.monotonic(); active = 'create_session'
    try:
        from .runtime import Session, HOME

        def make(force=False):
            return Session(str(asset_root), json.dumps(dict(physics_dt=DT, gripper_speed_mps=SPEED,
                gripper_force_hold=force, gripper_force_n=FORCE, initial_base_pose=[0, 0, .7, 0, 0, 0, 1])),
                contact_profile=contact_profile)

        def step(session, grip, targets=TARGET, dt=DT, allowed=True):
            _require(proof['step_count'] < 1500, 'Validation step budget exhausted')
            session.step(dt, targets, grip, allowed)
            proof['step_count'] += 1

        def passed(name, **evidence):
            proof['checks'].append(dict(name=name, passed=True, **evidence))

        session = make()
        proof['metadata'] = json.loads(session.metadata())
        _require(proof['metadata']['backend'] == 'Newton SolverMuJoCo CPU', 'Unexpected backend')
        active = 'target_speed_uses_physics_dt'
        rates = []
        for dt in (.0025, .005, .01):
            session.configure(json.dumps(dict(physics_dt=dt)))
            before = _targets(session)
            step(session, 1., HOME[:7], dt)
            delta = before-_targets(session)
            _require(session.np.allclose(delta, SPEED*dt, rtol=0, atol=1e-8), 'Target speed is not scaled by physics dt')
            rates.append(dict(dt=dt, target_change_m=delta.tolist()))
        passed(active, observations=rates)
        session.configure(json.dumps(dict(physics_dt=DT)))
        active = 'suppression_freezes_applied_setpoint'
        held = _targets(session)
        for _ in range(8):
            step(session, 0., HOME[:7], allowed=False)
            _require(session.np.array_equal(_targets(session), held), 'Suppression moved finger setpoint')
        passed(active, steps=8, held_target_q=held.tolist())
        active = 'mode_toggles_do_not_rebase_targets'
        for mode in (True, False, True):
            before = _targets(session)
            session.configure(json.dumps(dict(gripper_force_hold=mode)))
            _require(session.np.array_equal(before, _targets(session)), 'Mode change rebased target')
            step(session, 1., HOME[:7], allowed=False)
            _require(session.np.array_equal(before, _targets(session)), 'Suppressed mode transition moved target')
        passed(active)
        active = 'base_and_model_edits_retire_feedback_without_target_jump'
        operations = [
            ('base', lambda: session.set_base_pose([.001, 0, .7, 0, 0, 0, 1])),
            ('object', lambda: session.command(json.dumps(dict(version=1, op='spawn', id=9,
                pose=[2, 0, .8, 0, 0, 0, 1], half_extents=[.025]*3)))),
            ('room', lambda: session.set_environment(json.dumps(dict(version=1, revision=1, enabled=True,
                colliders=[dict(kind='floor', pose=[0, 0, -.025, 0, 0, 0, 1], half_extents=[3,3,.025])]))))]
        for name, operation in operations:
            step(session, 1., HOME[:7])
            before, index = _targets(session), session.step_index
            operation()
            _require(session.step_index == index and session.np.array_equal(before, _targets(session)), 'Edit changed time or target: '+name)
            step(session, 1., HOME[:7])
            state = json.loads(session.metadata())['gripper_control']
            _require(session.np.array_equal(before, _targets(session)) and state['phase'] == 'feedback_unavailable'
                     and state['measured_max_force_n'] is None, 'Edit reused invalidated force feedback: '+name)
        passed(active, edits=[name for name, _ in operations])
        active = 'invalid_gripper_settings_roll_back'
        before, settings = _targets(session), dict(session.settings)
        try:
            session.configure('{"gripper_force_n":21}')
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid force limit accepted')
        _require(settings == session.settings and session.np.array_equal(before, _targets(session)), 'Invalid settings changed targets')
        passed(active)

        def fixture(force_mode):
            s = make(force_mode)
            for i in range(250):
                alpha = min(1., (i+1)/200)
                arm = [a+(b-a)*alpha for a,b in zip(HOME[:7], TARGET)]
                step(s, 0., arm)
            poses, np = s.state.body_q.numpy(), s.np
            rotation = poses[9, 3:].copy()
            center = .5*(poses[10, :3]+poses[11, :3])+np.asarray(
                s.wp.quat_rotate(s.wp.quat(*rotation), s.wp.vec3(0, 0, .043)))
            top = float(center[2]-.025)
            room = dict(version=1, revision=1, enabled=True, colliders=[
                dict(kind='floor', pose=[0,0,-.025,0,0,0,1], half_extents=[3,3,.025]),
                dict(kind='table', pose=[float(center[0])+.27,float(center[1]),top-.025,0,0,0,1], half_extents=[.3,.3,.025])])
            s.set_environment(json.dumps(room))
            initial = [*map(float, center), 0,0,0,1]
            s.command(json.dumps(dict(version=1, op='spawn', id=1, pose=initial, half_extents=[.025]*3)))
            joint = s.model.joint_label.index('cube_1_free_joint')
            _require(s.model.joint_type.numpy()[joint] == s.newton.JointType.FREE
                     and s.model.joint_parent.numpy()[joint] == -1, 'Cube is not an independent free body')
            mass = float(s.model.body_mass.numpy()[s.object_bodies[1]])
            _require(abs(mass-.125) < 1e-7, 'Unexpected cube mass')
            return s, initial, dict(table_top_m=top, cube_home=initial, mass_kg=mass, fixed_base=True, room=room,
                approach_steps=250, close_steps=160, contact_physics=_physics_signature(s),
                initial_snapshot_sha256=hashlib.sha256(s.snapshot()[:40]+s.snapshot()[48:]).hexdigest())

        active = 'off_mode_reference_observation'
        baseline, initial, description = fixture(False)
        rows = []
        for i in range(160):
            step(baseline, (i+1)/160)
            rows.append(_sample(baseline, initial, 'close'))
        for _ in range(80):
            step(baseline, 1.)
            rows.append(_sample(baseline, initial, 'hold'))
        proof['baseline_off'] = dict(is_acceptance_gate=False, fixture=description, summary=_summary(rows), rows=rows)
        touched = any(any(row['positive_contacts']) for row in rows)
        proof['baseline_off']['lost_all_finger_contact_after_touch'] = touched and not any(rows[-1]['positive_contacts'])
        proof['baseline_off']['moved_over_half_cube_width_and_lost_contact'] = (
            rows[-1]['cube_horizontal_displacement_m'] > .025
            and proof['baseline_off']['lost_all_finger_contact_after_touch'])
        _require(_physics_signature(baseline) == description['contact_physics'], 'Baseline physics model changed')

        active = 'force_hold_supported_full_squeeze'
        held_session, initial, description = fixture(True)
        _require(description['initial_snapshot_sha256'] == proof['baseline_off']['fixture']['initial_snapshot_sha256'],
                 'Off and force-mode fixtures differ before squeeze')
        _require(description['contact_physics'] == proof['baseline_off']['fixture']['contact_physics'],
                 'Off and force-mode physics models differ')
        rows = []
        max_target_rate_excess = 0.
        for stage, count in (('close',160), ('hold',400)):
            for i in range(count):
                before = _targets(held_session)
                step(held_session, (i+1)/160 if stage == 'close' else 1.)
                change = float(held_session.np.max(held_session.np.abs(_targets(held_session)-before)))
                max_target_rate_excess = max(max_target_rate_excess, change-SPEED*DT)
                _require(change <= SPEED*DT+1e-8, 'Force controller exceeded target speed')
                rows.append(_sample(held_session, initial, stage))
        hold_rows = [row for row in rows if row['stage'] == 'hold']
        tail = _summary(hold_rows[-100:])
        proof['force_hold'] = dict(fixture=description, desired_force_n=FORCE, hold_steps=400,
            hold_seconds=2., summary=_summary(rows), final_half_second=tail, rows=rows,
            maximum_target_rate_excess_m=max_target_rate_excess)
        _require(_physics_signature(held_session) == description['contact_physics'], 'Force mode changed physics or attached cube')
        _require(tail['bilateral_samples'] >= 95, 'Supported hold did not retain bilateral finger contact')
        _require(tail['max_horizontal_displacement_m'] < .025, 'Supported cube moved more than half its width')
        _require(abs(tail['mean_max_opposing_force_n']-FORCE) <= 1., 'Opposing force did not converge to 5 N within 1 N')
        passed(active, final_half_second=tail, desired_force_n=FORCE, convergence_tolerance_n=1.,
                horizontal_displacement_limit_m=.025)

        active = 'loaded_room_rebuild_does_not_reuse_previous_contact_force'
        old_force = rows[-1]['opposing_force_n']
        _require(max(old_force) > 1., 'Loaded invalidation probe lacks nonzero prior contact')
        before, before_step = _targets(held_session), held_session.step_index
        held_session.set_environment(json.dumps(dict(description['room'], revision=2)))
        _require(held_session.step_index == before_step
                 and held_session.np.array_equal(before, _targets(held_session)), 'Loaded room edit changed time or target')
        step(held_session, 1.)
        state = json.loads(held_session.metadata())['gripper_control']
        _require(held_session.np.array_equal(before, _targets(held_session))
                 and state['phase'] == 'feedback_unavailable' and state['measured_max_force_n'] is None,
                 'First solve after loaded room edit reused previous contact forces')
        _require(_physics_signature(held_session) == description['contact_physics'], 'Identical room refresh changed physical model')
        passed(active, prior_opposing_force_n=old_force, first_step_phase=state['phase'], held_target_q=before.tolist())

        active = 'release_opens_and_relinquishes_finger_contact'
        release = []
        for _ in range(120):
            step(held_session, 0.)
            release.append(_sample(held_session, initial, 'release'))
        final = release[-1]
        opened = all(q > .039 for q in final['target_q']) and all(q > .038 for q in final['measured_q'])
        no_contact = all(not any(row['positive_contacts']) for row in release[-20:])
        proof['release'] = dict(steps=120, seconds=.6, opened_and_lost_finger_contact=opened and no_contact,
                               summary=_summary(release), rows=release)
        _require(opened and no_contact, 'Release failed to open fingers and lose contacts')
        _require(final['static_normal_force_n'] > 0, 'Released cube not table-supported')
        expected_z=description['table_top_m']+.025
        _require(abs(final['cube_xyz'][2]-expected_z)<.01, 'Released cube is not at table height')
        _require(_physics_signature(held_session) == description['contact_physics'], 'Release changed physical constraints')
        passed(active, final_target_q=final['target_q'], final_measured_q=final['measured_q'],
                last_twenty_samples_without_finger_contact=True,
                final_cube_center_z_m=final['cube_xyz'][2],expected_table_center_z_m=expected_z,
                table_height_tolerance_m=.01)
        proof['final_metadata'] = json.loads(held_session.metadata())
        proof['diagnostic_checks'] = 'Native normal-load decoder equals mj_contactForce; 10 Hz QDIA retains loaded fingers in the single-cube fixture'
        proof['passed'] = True
    except Exception as error:
        proof['error'] = dict(check=active, type=type(error).__name__, message=str(error), traceback=traceback.format_exc())
    proof['validation_elapsed_seconds'] = time.monotonic()-started
    return json.dumps(proof, allow_nan=False)
