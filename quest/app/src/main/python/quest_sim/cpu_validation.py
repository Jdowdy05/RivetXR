"""Portable exact CPU replay for the host checker and opt-in Android proof.

No subprocesses, host paths, timing benchmarks, or device control. The caller
owns evidence persistence and startup rejection. Compare named active contact
fields, never struct padding; only QSIM CPU-time bytes 40:48 are excluded.
"""
from __future__ import annotations

import collections
import hashlib
import json
import math
import platform
import struct
import sys
import time
import traceback


HOME = [0., -.569, 0., -2.810, 0., 3.037, .741]
STATE_ARRAYS = ('joint_q', 'joint_qd', 'body_q', 'body_qd', 'body_f', 'body_qdd',
                'body_parent_f', 'particle_q', 'particle_qd', 'particle_f')
CONTROL_ARRAYS = ('joint_f', 'joint_target_q', 'joint_target_qd', 'joint_act')
MODEL_ARRAYS = ('joint_X_p', 'joint_X_c', 'joint_q', 'joint_qd', 'body_q',
                'joint_target_q', 'joint_target_qd', 'joint_target_ke', 'joint_target_kd')
MJ_DATA_ARRAYS = ('qpos', 'qvel', 'qacc', 'qacc_warmstart', 'act', 'act_dot', 'ctrl',
                  'mocap_pos', 'mocap_quat', 'qfrc_applied', 'xfrc_applied',
                  'qfrc_actuator', 'qfrc_constraint', 'qfrc_bias', 'qfrc_passive',
                  'xpos', 'xquat', 'cvel', 'efc_force', 'efc_state')
MJ_MODEL_ARRAYS = ('body_pos', 'body_quat', 'jnt_pos', 'jnt_axis', 'qpos0')
CONTACT_FIELDS = ('dist', 'pos', 'frame', 'includemargin', 'friction', 'solref',
                  'solreffriction', 'solimp', 'mu', 'H', 'dim', 'geom', 'geom1',
                  'geom2', 'flex', 'elem', 'vert', 'exclude', 'efc_address', 'adhesion')


def snapshot_physics_bytes(data):
    """Validate QSIM framing and exclude exactly its nondeterministic CPU time."""
    if len(data) < 48:
        raise ValueError('Short QSIM snapshot')
    magic, version, header, bodies, objects = struct.unpack_from('<4sHHII', data)
    if magic != b'QSIM' or version != 1 or header != 48 or len(data) != 48 + bodies * 28 + objects * 20:
        raise ValueError('Invalid QSIM snapshot framing')
    return data[:40] + data[48:]


def _description(value):
    if isinstance(value, bytes):
        return dict(bytes=len(value), sha256=hashlib.sha256(value).hexdigest(), prefix_hex=value[:32].hex())
    if isinstance(value, (tuple, list)):
        return [_description(v) for v in value]
    return value


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite': 'NaN' if math.isnan(value) else '+Infinity' if value > 0 else '-Infinity'}
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


def first_difference(reference, candidate, path=''):
    """Return the first exact mismatch, including dtype/shape and float bits."""
    if type(reference) is not type(candidate):
        return dict(field=path, reference_type=type(reference).__name__, candidate_type=type(candidate).__name__)
    if isinstance(reference, dict):
        if reference.keys() != candidate.keys():
            return dict(field=path, missing=sorted(reference.keys() - candidate.keys()),
                        extra=sorted(candidate.keys() - reference.keys()))
        for key in sorted(reference):
            difference = first_difference(reference[key], candidate[key], path + '.' + key)
            if difference:
                return difference
        return None
    if isinstance(reference, (tuple, list)):
        if len(reference) != len(candidate):
            return dict(field=path, reference_length=len(reference), candidate_length=len(candidate))
        for index, (a, b) in enumerate(zip(reference, candidate)):
            difference = first_difference(a, b, f'{path}[{index}]')
            if difference:
                return difference
        return None
    if isinstance(reference, float):
        reference, candidate = struct.pack('<d', reference), struct.pack('<d', candidate)
    if reference != candidate:
        result = dict(field=path, reference=_description(reference), candidate=_description(candidate))
        if isinstance(reference, bytes):
            result['first_differing_byte'] = next((i for i, (a, b) in enumerate(zip(reference, candidate)) if a != b),
                                                   min(len(reference), len(candidate)))
        return result
    return None


def _array(value):
    if value is None:
        return None
    import numpy as np
    array = value.numpy() if hasattr(value, 'numpy') else np.asarray(value)
    return (array.dtype.str, tuple(array.shape), array.tobytes(order='C'))


def capture_session(session):
    """Copy semantic state fields, never C-struct padding or inactive contacts."""
    captured = {}
    for label, owner, fields in (
            ('state', session.state, STATE_ARRAYS), ('next_state', session.next_state, STATE_ARRAYS),
            ('control', session.control, CONTROL_ARRAYS), ('model', session.model, MODEL_ARRAYS),
            ('mujoco', session.solver.mj_data, MJ_DATA_ARRAYS),
            ('mujoco_model', session.solver.mj_model, MJ_MODEL_ARRAYS)):
        for name in fields:
            captured[label + '.' + name] = _array(getattr(owner, name, None))
    for name in ('mocap_pos', 'mocap_quat'):
        captured['mjwarp.' + name] = _array(getattr(session.solver.mjw_data, name))
    for name in ('body_pos', 'body_quat', 'jnt_pos', 'jnt_axis'):
        captured['mjwarp_model.' + name] = _array(getattr(session.solver.mjw_model, name))
    data = session.solver.mj_data
    captured['ncon'] = int(data.ncon)
    captured['nefc'] = int(data.nefc)
    for name in CONTACT_FIELDS:
        value = getattr(data.contact, name, None)
        captured['contact.' + name] = _array(value[:data.ncon]) if value is not None else None
    captured['session'] = dict(step_index=session.step_index, sim_time=session.sim_time,
                               generation=session.generation, base_pose=list(session.base_pose),
                               settings=dict(session.settings), has_box=session.has_box,
                               mujoco_time=float(data.time))
    return captured


def operation_schedule(steps):
    """Deterministic inputs; no target is derived from candidate output."""
    operations = []
    def add(kind, **values):
        operations.append(dict(kind=kind, **values))
    def motion(count, dt, floating=False):
        for index in range(count):
            phase = index * .04
            if not floating and index % 2 == 0:
                yaw = .08 * math.sin(phase * .5)
                pose = [.02 * math.sin(phase), -.01 * math.sin(phase * .7),
                        .7 + .02 * math.sin(phase * .5), 0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]
                add('base', pose=pose)
                if index % 32 == 0:
                    add('base', pose=pose)  # Identical repeated setter, no intervening step.
            targets = [home + .015 * math.sin(phase + joint * .3) for joint, home in enumerate(HOME)]
            add('step', dt=dt, targets=targets, gripper=.5 - .5 * math.cos(phase))
    add('command', name='spawn_box')
    motion(steps, .005)
    add('forces', enabled=True)
    add('solver_step', dt=.005)  # Direct backend probe preserves nonzero body forces.
    add('forces', enabled=True)
    add('step', dt=.005, targets=HOME, gripper=.7)  # Production force-clear behavior.
    add('forces', enabled=False)
    add('command', name='reset')
    add('base', pose=[.1, -.1, .9, 0., 0., math.sin(.15), math.cos(.15)])
    add('snapshot')  # Standalone/paused root update, with no solver step.
    add('command', name='remove_box')
    add('command', name='spawn_box')
    add('configure', settings={'physics_dt': .0025, 'control_decimation': 4, 'render_interval': 4})
    motion(max(16, steps // 4), .0025)
    add('configure', settings={'gravity_scale': .5})
    motion(max(16, steps // 4), .0025)
    add('configure', settings={'floating_base': True})
    add('base', pose=[0., 0., 1., 0., 0., 0., 1.], expect_error='ValueError')
    motion(max(32, steps // 2), .0025, floating=True)
    add('forces', enabled=True)
    add('solver_step', dt=.0025)
    add('forces', enabled=False)
    add('command', name='reset')
    add('command', name='remove_box')
    add('command', name='spawn_box')
    add('configure', settings={'floating_base': False, 'gravity_scale': 1., 'physics_dt': .005})
    for bad in (float('nan'), float('inf')):
        add('base', pose=[bad, 0., .7, 0., 0., 0., 1.], expect_error='ValueError')
        add('step', dt=.005, targets=[bad, *HOME[1:]], gripper=.5, expect_error='ValueError')
        add('step', dt=.005, targets=HOME, gripper=bad, expect_error='ValueError')
    add('step', dt=.01, targets=HOME, gripper=.5, expect_error='ValueError')
    add('configure', settings={'render_interval': 0}, expect_error='ValueError')
    for array, value in (('joint_q', float('nan')), ('joint_qd', float('inf')), ('body_q', float('nan'))):
        add('inject', array=array, value=value)
        add('snapshot', expect_error='FloatingPointError')
        add('command', name='reset')
    motion(16, .005)
    return operations


def apply_operation(session, operation):
    kind = operation['kind']
    if kind == 'base':
        return session.set_base_pose(operation['pose'])
    if kind == 'step':
        return session.step(operation['dt'], operation['targets'], operation['gripper'])
    if kind == 'command':
        return session.command(operation['name'])
    if kind == 'configure':
        session.configure(json.dumps(operation['settings']))
        return session.snapshot()
    if kind == 'snapshot':
        return session.snapshot()
    if kind == 'forces':
        body = session.state.body_f.numpy()
        joint = session.control.joint_f.numpy()
        body[:] = 0; joint[:] = 0
        if operation['enabled']:
            body[0] = [.1, -.2, .3, 1., -2., 3.]
            body[-1] = [-.2, .1, .05, .3, -.4, .5]  # Dynamic box, not only the prescribed root.
            joint[session.arm_dof_indices[0]] = .25
            joint[session.arm_dof_indices[-1]] = -.1
        return None
    if kind == 'inject':
        array = getattr(session.state, operation['array']).numpy()
        # Last joint coordinate includes the uncommanded free box when present.
        array.reshape(-1)[-1] = operation['value']
        return None
    if kind == 'solver_step':
        session.solver.step(session.state, session.next_state, session.control, None, operation['dt'])
        session.state, session.next_state = session.next_state, session.state
        session.step_index += 1; session.sim_time += operation['dt']
        session.step_cpu_ms = 0.
        return session.snapshot()
    raise ValueError('Unknown replay operation: ' + kind)


def outcome(session, operation):
    try:
        value = apply_operation(session, operation)
        error = None
    except Exception as exception:
        value = None
        error = (type(exception).__name__, str(exception))
    return dict(error=error, snapshot=snapshot_physics_bytes(value) if isinstance(value, bytes) else None)


def compare_replay(reference_factory, candidate_factory, assets, operations):
    initial = json.dumps(dict(physics_dt=.005, initial_base_pose=[0., 0., .7, 0., 0., 0., 1.]))
    reference, candidate = reference_factory(str(assets), initial), candidate_factory(str(assets), initial)
    comparison_count = 0
    digest = hashlib.sha256()
    execution_segments = []
    last_generation = None
    for index, operation in enumerate([None, *operations]):
        if operation and operation['kind'] in ('configure', 'command'):
            execution_segments.append(dict(operation_index=index, boundary='before', generation=candidate.generation,
                                            metadata=json.loads(candidate.metadata())))
        results = ({}, {}) if operation is None else (outcome(reference, operation), outcome(candidate, operation))
        if operation is not None:
            expected = operation.get('expect_error')
            actual = results[0]['error'][0] if results[0]['error'] else None
            if actual != expected:
                return dict(passed=False, operation_index=index, operation=operation,
                            reason='Reference outcome disagrees with replay contract',
                            expected_error=expected, reference=results[0]['error'])
        difference = first_difference(results[0], results[1], 'outcome')
        if difference is None:
            a, b = capture_session(reference), capture_session(candidate)
            comparison_count += len(a)
            difference = first_difference(a, b, 'state')
            for key, value in sorted(a.items()):
                digest.update(key.encode()); digest.update(repr(value).encode())
        if difference:
            return dict(passed=False, operation_index=index, operation=operation,
                        difference=difference, field_comparisons=comparison_count,
                        candidate_metadata=json.loads(candidate.metadata()), execution_segments=execution_segments)
        if last_generation != candidate.generation or index == len(operations):
            execution_segments.append(dict(operation_index=index, boundary='after', generation=candidate.generation,
                                            metadata=json.loads(candidate.metadata())))
            last_generation = candidate.generation
        elif operation and operation['kind'] in ('configure', 'command', 'solver_step'):
            execution_segments.append(dict(operation_index=index, boundary='after', generation=candidate.generation,
                                            metadata=json.loads(candidate.metadata())))
    return dict(passed=True, operations=len(operations), field_comparisons=comparison_count,
                reference_state_digest=digest.hexdigest(), metadata=json.loads(reference.metadata()),
                candidate_metadata=json.loads(candidate.metadata()), execution_segments=execution_segments)



def validate(reference_factory, candidate_factory, assets, steps=256):
    """Return a complete success/failure proof; a failed control stops comparison."""
    proof = dict(schema_version=1, passed=False,
                 scope='Exact CPU comparison on this process/platform; no cross-platform or performance claim',
                 platform=sys.platform, machine=platform.machine(), python=sys.version,
                 comparison='Exact dtype, shape and bytes; only snapshot CPU time bytes40:48 excluded',
                 contact_policy='All named fields of active contacts; no C-struct padding or inactive capacity',
                 main_steps=steps)
    started = time.monotonic()
    try:
        if type(steps) is not int or not 8 <= steps <= 2048:
            raise ValueError('CPU validation main steps must be an integer in [8,2048]')
        operations = operation_schedule(steps)
        proof['operation_counts'] = dict(collections.Counter(op['kind'] for op in operations))
        proof['operations_sha256'] = hashlib.sha256(json.dumps(operations, sort_keys=True).encode()).hexdigest()
        proof['reference_control'] = compare_replay(reference_factory, reference_factory, assets, operations)
        if proof['reference_control']['passed']:
            proof['candidate_comparison'] = compare_replay(reference_factory, candidate_factory, assets, operations)
            proof['passed'] = proof['candidate_comparison']['passed']
    except Exception as exception:
        proof['passed'] = False
        proof['error'] = dict(type=type(exception).__name__, message=str(exception), traceback=traceback.format_exc())
    proof['validation_elapsed_seconds'] = time.monotonic() - started
    return json_safe(proof)


def run(asset_root, steps=256):
    """Android bridge entry: return proof JSON for persistence before acceptance."""
    def reference(assets, settings):
        from .runtime import Session
        return Session(assets, settings, cpu_cache=False)
    def candidate(assets, settings):
        from .runtime import Session
        return Session(assets, settings, cpu_cache=True)
    proof = validate(reference, candidate, asset_root, steps)
    proof['reference_scope'] = 'Current Session wrapper with upstream Newton CPU solver'
    proof['candidate_scope'] = 'Current Session wrapper with cached Newton CPU solver'
    return json.dumps(proof, allow_nan=False)
