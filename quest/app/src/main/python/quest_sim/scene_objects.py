"""Strict user-cube edits and identity-based transfer of the pinned CPU scene."""
from dataclasses import dataclass
import json
import math
import struct

from .room_environment import MIN_HALF_EXTENT, _fields, _unique_object, _vector

MAX_USER_OBJECTS = 8
MAX_HALF_EXTENT = struct.unpack("<f", struct.pack("<f", .1))[0]


@dataclass(frozen=True)
class SceneObject:
    id: int
    home_pose: tuple[float, ...]
    half_extents: tuple[float, ...]

    @property
    def label(self):
        return f"cube_{self.id}"


@dataclass(frozen=True)
class ObjectCommand:
    op: str
    id: int
    pose: tuple[float, ...] | None = None
    half_extents: tuple[float, ...] | None = None


def parse_object_command(raw):
    value = json.loads(raw, object_pairs_hook=_unique_object)
    if not isinstance(value, dict):
        raise ValueError("object command must be a JSON object")
    op = value.get("op")
    if op not in ("spawn", "remove", "reset", "move"):
        raise ValueError("object op must be spawn, remove, reset or move")
    fields = ("version", "op", "id")
    if op in ("spawn", "move"):
        fields += ("pose",)
    if op == "spawn":
        fields += ("half_extents",)
    _fields(value, fields, "object command")
    if type(value["version"]) is not int or value["version"] != 1:
        raise ValueError("object command version must be 1")
    if type(value["id"]) is not int or not 1 <= value["id"] < 2**32:
        raise ValueError("object id must be a positive uint32 integer")
    pose = extents = None
    if "pose" in value:
        pose = _vector(value["pose"], 7, "object pose")
        if any(abs(v) > 100. for v in pose[:3]) or abs(math.hypot(*pose[3:])-1.) > 1e-3:
            raise ValueError("object pose requires XYZ within +/-100 metres and a unit XYZW quaternion")
    if "half_extents" in value:
        extents = _vector(value["half_extents"], 3, "object half_extents")
        if any(not MIN_HALF_EXTENT <= v <= MAX_HALF_EXTENT for v in extents) or len(set(extents)) != 1:
            raise ValueError("cube half extents must be equal in [float32(.005), float32(.1)] metres")
    return ObjectCommand(op, value["id"], pose, extents)


def apply_object_command(session, raw):
    command = parse_object_command(raw)
    selected = next((obj for obj in session.objects if obj.id == command.id), None)
    objects, used_ids, poses, reset_labels = session.objects, session.used_object_ids, {}, set()
    if command.op == "spawn":
        if command.id in used_ids:
            raise ValueError("object id was already used in this Session")
        if len(objects) >= MAX_USER_OBJECTS:
            raise ValueError("at most eight user cubes may be active")
        objects += (SceneObject(command.id, command.pose, command.half_extents),)
        used_ids = used_ids | {command.id}
    else:
        if selected is None:
            raise ValueError("unknown user object id")
        if command.op == "remove":
            objects = tuple(obj for obj in objects if obj.id != command.id)
        else:
            poses[command.id] = command.pose if command.op == "move" else selected.home_pose
            reset_labels.add(selected.label)
    replacement = object.__new__(type(session))
    replacement.__dict__.update(session.__dict__)
    replacement.objects, replacement.used_object_ids = objects, used_ids
    replacement._build()
    restore_object_state(replacement, session, reset_labels, poses)
    replacement._reset_gripper_control()
    snapshot = replacement.snapshot()
    replacement.details_bytes(False)  # Validate the matching identity publication before commit.
    session.__dict__.update(replacement.__dict__)
    return snapshot


def _labels(values, name):
    result = {label: index for index, label in enumerate(values)}
    if len(result) != len(values):
        raise ValueError(f"non-unique {name} labels")
    return result


def _warp_arrays(owner, wp, namespace_type, prefix=""):
    result = {}
    for name, value in vars(owner).items():
        if isinstance(value, wp.array):
            result[prefix+name] = value
        elif isinstance(value, namespace_type):
            result.update(_warp_arrays(value, wp, namespace_type, prefix+name+"."))
    return result


def _transfer_warp(owner, destination, source, body_ranges, q_ranges, dof_ranges, control=False):
    old = _warp_arrays(source, owner.wp, owner.newton.Model.AttributeNamespace)
    new = _warp_arrays(destination, owner.wp, owner.newton.Model.AttributeNamespace)
    if set(old) != set(new):
        raise ValueError("object edit changed allocated state/control attributes")
    for name, source_array in old.items():
        target = new[name]
        if source_array.dtype != target.dtype or source_array.shape[1:] != target.shape[1:]:
            raise ValueError(f"object edit changed array type for {name}")
        if source_array.size == target.size == 0:
            continue
        if control:
            if name == "joint_target_q":
                ranges = q_ranges if owner.model.use_coord_layout_targets else dof_ranges
            elif name in ("joint_target_qd", "joint_f", "joint_act"):
                ranges = dof_ranges
            elif name == "mujoco.ctrl":
                if target.shape != source_array.shape:
                    raise ValueError("object edit changed custom actuator controls")
                target.assign(source_array)
                continue
            else:
                raise ValueError(f"unsupported topology transfer control array: {name}")
        elif name in ("body_q", "body_qd", "body_f", "body_qdd", "body_parent_f", "_deprecated_body_q_prev"):
            ranges = body_ranges
        elif name == "joint_q":
            ranges = q_ranges
        elif name in ("joint_qd", "mujoco.qfrc_actuator"):
            ranges = dof_ranges
        else:
            raise ValueError(f"unsupported topology transfer state array: {name}")
        values, before = target.numpy(), source_array.numpy()
        for old_start, new_start, count in ranges:
            values[new_start:new_start+count] = before[old_start:old_start+count]
        target.assign(values)


def _mj_bodies(session):
    result = {}
    for index, body in enumerate(session.solver.mjc_body_to_newton.numpy()[0]):
        label = session.model.body_label[body] if body >= 0 else "@world"
        if label in result:
            raise ValueError("non-unique native body mapping")
        result[label] = index
    return result


def restore_object_state(current, previous, reset_labels, object_poses):
    """Copy survivors in their original precision; fresh contacts stay model-local."""
    np = current.np
    old, new = previous.solver, current.solver
    old_model, new_model = previous.model, current.model
    if (current.settings != previous.settings or current.environment != previous.environment
            or old_model.use_coord_layout_targets != new_model.use_coord_layout_targets
            or old_model.world_count != 1 or new_model.world_count != 1
            or old_model.body_label[:12] != new_model.body_label[:12]):
        raise ValueError("object edit changed robot, world or solver settings")
    old_bodies, new_bodies = _labels(old_model.body_label, "body"), _labels(new_model.body_label, "body")
    old_joints, new_joints = _labels(old_model.joint_label, "joint"), _labels(new_model.joint_label, "joint")
    old_ids, new_ids = {obj.id for obj in previous.objects}, {obj.id for obj in current.objects}
    removed = {obj.label for obj in previous.objects if obj.id not in new_ids}
    added = {obj.label for obj in current.objects if obj.id not in old_ids}
    if (old_bodies.keys()-new_bodies.keys() != removed or new_bodies.keys()-old_bodies.keys() != added
            or old_joints.keys()-new_joints.keys() != {label+"_free_joint" for label in removed}
            or new_joints.keys()-old_joints.keys() != {label+"_free_joint" for label in added}):
        raise ValueError("object edit changed an unexpected body or joint identity")
    body_ranges = [(index, new_bodies[label], 1) for label, index in old_bodies.items()
                   if label in new_bodies and label not in reset_labels]
    old_q, new_q = old_model.joint_q_start.numpy(), new_model.joint_q_start.numpy()
    old_dof, new_dof = old_model.joint_qd_start.numpy(), new_model.joint_qd_start.numpy()
    old_type, new_type = old_model.joint_type.numpy(), new_model.joint_type.numpy()
    old_child, new_child = old_model.joint_child.numpy(), new_model.joint_child.numpy()
    old_parent, new_parent = old_model.joint_parent.numpy(), new_model.joint_parent.numpy()
    old_mjq, new_mjq = old.mj_q_start.numpy(), new.mj_q_start.numpy()
    old_mjd, new_mjd = old.mj_qd_start.numpy(), new.mj_qd_start.numpy()
    q_ranges, dof_ranges, native_q, native_dof = [], [], [], []
    def body_name(model, index):
        return model.body_label[index] if index >= 0 else "@world"
    for label, before in old_joints.items():
        if label not in new_joints or body_name(old_model, old_child[before]) in reset_labels:
            continue
        after = new_joints[label]
        nq, nv = int(old_q[before+1]-old_q[before]), int(old_dof[before+1]-old_dof[before])
        if (old_type[before] != new_type[after] or nq != new_q[after+1]-new_q[after]
                or nv != new_dof[after+1]-new_dof[after]
                or body_name(old_model, old_child[before]) != body_name(new_model, new_child[after])
                or body_name(old_model, old_parent[before]) != body_name(new_model, new_parent[after])):
            raise ValueError(f"object edit changed joint topology: {label}")
        q_ranges.append((int(old_q[before]), int(new_q[after]), nq))
        dof_ranges.append((int(old_dof[before]), int(new_dof[after]), nv))
        native_q.append((int(old_mjq[before]), int(new_mjq[after]), nq))
        native_dof.append((int(old_mjd[before]), int(new_mjd[after]), nv))
    # Franka's 18 actuators are unnamed. Its known unchanged mapping, not names,
    # establishes that ctrl/activation slots still belong to the same robot.
    for name in ("mjc_actuator_ctrl_source", "mjc_actuator_to_newton_idx", "mjc_actuator_to_newton_target_q_idx",
                 "mjc_actuator_to_target_q_axis_idx", "mjc_actuator_to_newton_ball_jnt", "mjc_mocap_to_newton_jnt"):
        a, b = getattr(old, name), getattr(new, name)
        if (a is None) != (b is None) or (a is not None and not np.array_equal(a.numpy(), b.numpy())):
            raise ValueError(f"object edit changed actuator mapping: {name}")
    for name in ("actuator_trntype", "actuator_trnid", "actuator_dyntype", "actuator_actadr", "actuator_actnum",
                 "eq_type", "eq_objtype", "eq_obj1id", "eq_obj2id"):
        if not np.array_equal(getattr(old.mj_model, name), getattr(new.mj_model, name)):
            raise ValueError(f"object edit changed robot actuator/equality layout: {name}")
    for name in ("nu", "na", "neq", "nmocap", "nuserdata", "npluginstate"):
        if getattr(old.mj_model, name) != getattr(new.mj_model, name):
            raise ValueError(f"object edit changed native {name}")
    for name in ("history", "userdata", "plugin_state"):
        if getattr(old.mj_data, name).size or getattr(new.mj_data, name).size:
            raise ValueError(f"unsupported nonempty topology transfer field: {name}")
    for name, ranges in (("qpos", native_q), ("qvel", native_dof), ("qacc_warmstart", native_dof), ("qfrc_applied", native_dof)):
        source, target = getattr(old.mj_data, name), getattr(new.mj_data, name)
        for old_start, new_start, count in ranges:
            target[new_start:new_start+count] = source[old_start:old_start+count]
    old_native, new_native = _mj_bodies(previous), _mj_bodies(current)
    for label, old_index in old_native.items():
        if label in new_native and label not in reset_labels:
            new.mj_data.xfrc_applied[new_native[label]] = old.mj_data.xfrc_applied[old_index]
    for name in ("ctrl", "act", "mocap_pos", "mocap_quat", "eq_active"):
        getattr(new.mj_data, name)[:] = getattr(old.mj_data, name)
    for name in ("mocap_pos", "mocap_quat"):
        getattr(new.mjw_data, name).assign(getattr(old.mjw_data, name))
    _transfer_warp(current, current.state, previous.state, body_ranges, q_ranges, dof_ranges)
    _transfer_warp(current, current.next_state, previous.next_state, body_ranges, q_ranges, dof_ranges)
    _transfer_warp(current, current.control, previous.control, body_ranges, q_ranges, dof_ranges, control=True)
    # A move changes live state, never model defaults: full Reset still returns
    # each cube to its original spawn pose. User cubes are COM-centred free joints
    # with identity anchors, verified here before applying XYZ/XYZW coordinates.
    for object_id, pose in object_poses.items():
        label = f"cube_{object_id}"
        joint, body = new_joints[label+"_free_joint"], new_bodies[label]
        identity = np.array([0,0,0,0,0,0,1], dtype=np.float32)
        if (new_type[joint] != current.newton.JointType.FREE or new_parent[joint] != -1
                or not np.array_equal(new_model.joint_X_p.numpy()[joint], identity)
                or not np.array_equal(new_model.joint_X_c.numpy()[joint], identity)
                or np.any(new_model.body_com.numpy()[body])):
            raise ValueError("user cube free-joint pose contract changed")
        value = np.asarray(pose, dtype=np.float32)
        for state in (current.state, current.next_state):
            coordinates, transforms = state.joint_q.numpy(), state.body_q.numpy()
            coordinates[new_q[joint]:new_q[joint+1]] = value
            transforms[body] = value
            state.joint_q.assign(coordinates); state.body_q.assign(transforms)
        start = int(new_mjq[joint])
        new.mj_data.qpos[start:start+7] = (*value[:3], value[6], *value[3:6])
    new.mj_data.time, new._step = old.mj_data.time, old._step
    new.mj_model.opt.timestep = old.mj_model.opt.timestep
    current.step_index, current.sim_time, current.step_cpu_ms = previous.step_index, previous.sim_time, previous.step_cpu_ms
    new._mujoco.mj_forward(new.mj_model, new.mj_data)
