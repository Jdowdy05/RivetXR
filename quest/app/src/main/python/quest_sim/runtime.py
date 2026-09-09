"""Newton CPU scene, owned exclusively by the native simulation worker.

Snapshot v1 is little endian: <4sHHIIIIQdd (48 bytes), then body_count
transforms of 7 float32 (xyz, xyzw), then object_count <II3f records
(body index, kind=1 box, half extents). The header contains QSIM, version,
header bytes, body count, object count, generation, contacts, step index,
simulation seconds and last-step CPU milliseconds. All poses are Z-up metres.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as ET

from .room_environment import RoomEnvironment, parse_environment
from .contact_model import DEFAULT_PROFILE, resolve_profile, apply_profile, native_metadata

HEADER = struct.Struct("<4sHHIIIIQdd")
OBJECT = struct.Struct("<II3f")
DEFAULTS = dict(physics_dt=0.005, control_decimation=2, render_interval=2,
                floating_base=False, gravity_scale=1.0,
                # Raw Python/legacy references opt in explicitly. Native live
                # settings always supply the user-selected positive speed.
                gripper_force_hold=False, gripper_speed_mps=0., gripper_force_n=5.,
                contact_profile=DEFAULT_PROFILE)
ARM_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
JOINT_NAMES = ARM_NAMES + ["panda_finger_joint1", "panda_finger_joint2"]
BODY_NAMES = [f"panda_link{i}" for i in range(9)] + ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
HOME = [0, -0.569, 0, -2.810, 0, 3.037, 0.741, 0.04, 0.04]
GAINS = [400] * 7 + [2000] * 2
DAMPING = [80] * 7 + [100] * 2


def bootstrap(native_library_dir: str, cache_dir: str):
    """Call before create on Android, with actual app-private directory paths."""
    os.environ["WARP_NATIVE_LIBRARY_DIR"] = native_library_dir
    os.environ["WARP_CACHE_PATH"] = cache_dir
    os.environ["MUJOCO_GL"] = "disable"
    Path(cache_dir).mkdir(parents=True, exist_ok=True)


def _settings(raw, previous=None):
    result = dict(DEFAULTS if previous is None else previous)
    incoming = json.loads(raw)
    if not isinstance(incoming, dict):
        raise ValueError("settings must be a JSON object")
    for key in DEFAULTS:
        if key in incoming:
            result[key] = incoming[key]
    for key, low, high in (("physics_dt", 0.0005, 0.02), ("gravity_scale", 0.0, 2.0)):
        value = result[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be finite in [{low}, {high}]")
    for key in ("control_decimation", "render_interval"):
        if type(result[key]) is not int or not 1 <= result[key] <= 40:
            raise ValueError(f"{key} must be an integer in [1, 40]")
    if type(result["floating_base"]) is not bool:
        raise ValueError("floating_base must be boolean")
    if type(result['gripper_force_hold']) is not bool:
        raise ValueError('gripper_force_hold must be boolean')
    speed, force = result['gripper_speed_mps'], result['gripper_force_n']
    if (isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed)
            or not (speed == 0. or .005 <= speed <= .2)):
        raise ValueError('gripper_speed_mps must be zero (legacy) or in [.005,.2]')
    if isinstance(force, bool) or not isinstance(force, (int, float)) or not math.isfinite(force) or not .5 <= force <= 20.:
        raise ValueError('gripper_force_n must be in [.5,20]')
    if result['gripper_force_hold'] and speed == 0.:
        raise ValueError('force holding requires positive gripper speed')
    resolve_profile(result['contact_profile'])
    return result


def create(asset_root: str, settings_json: str):
    return Session(asset_root, settings_json)


class Session:
    def __init__(self, asset_root, settings_json, *, cpu_cache=True, contact_profile=None):
        settings = _settings(settings_json)
        if contact_profile is not None:
            chosen = resolve_profile(contact_profile)
            incoming = json.loads(settings_json)
            if 'contact_profile' in incoming and settings['contact_profile'] != chosen.name:
                raise ValueError('Conflicting settings and diagnostic contact profiles')
            settings['contact_profile'] = chosen.name
        self._contact_profile = resolve_profile(settings['contact_profile'])
        # Heavy imports are deliberately confined to the worker's create call.
        import numpy as np
        import warp as wp
        import newton
        self.np, self.wp, self.newton = np, wp, newton
        # Diagnostic reference sessions run the original upstream dispatch.
        # This is not a persisted simulation setting or a physics variant.
        if type(cpu_cache) is not bool:
            raise ValueError("cpu_cache must be boolean")
        self.cpu_cache = cpu_cache
        wp.init()
        wp.set_device("cpu")
        self.asset_root = Path(asset_root)
        self.settings = settings
        initial_pose = json.loads(settings_json).get("initial_base_pose", [0., 0., 0., 0., 0., 0., 1.])
        self.base_pose = self._validated_pose(initial_pose).tolist()
        self.has_box = False
        self.objects = ()
        self.used_object_ids = frozenset()
        self.environment = RoomEnvironment()
        self.generation = 0
        self._build()

    def _build(self):
        np, wp, newton = self.np, self.wp, self.newton
        self._contact_profile = resolve_profile(self.settings['contact_profile'])
        from .cpu_solver import CpuSolverMuJoCo
        from .cpu_views import CpuViewCache
        # _rebuild shallow-copies the old session; replace its cache rather than
        # clearing a shared cache and disturbing the still-valid old scene.
        self._cpu_views = CpuViewCache() if self.cpu_cache else None
        root = self.asset_root
        if (root / "franka_description").is_dir():
            root = root / "franka_description"
        tree = ET.parse(root / "robots/panda_arm_hand.urdf")
        # Native renderer owns visual meshes. Retain every collision/inertial/joint.
        for link in tree.getroot().findall("link"):
            for visual in link.findall("visual"):
                link.remove(visual)
        for mesh in tree.getroot().iter("mesh"):
            relative = mesh.attrib["filename"].removeprefix("package://franka_description/")
            filename = (root / relative).resolve()
            if not filename.is_file():
                raise FileNotFoundError(filename)
            mesh.set("filename", filename.as_posix())
        builder = newton.ModelBuilder(gravity=(0., 0., -9.81 * self.settings["gravity_scale"]))
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
        builder.add_urdf(ET.tostring(tree.getroot(), encoding="unicode"),
                         floating=self.settings["floating_base"],
                         xform=wp.transform(self.base_pose[:3], self.base_pose[3:]),
                         enable_self_collisions=True, collapse_fixed_joints=False,
                         force_position_velocity_actuation=True)
        builder.joint_label[:] = [s.removeprefix("panda/") for s in builder.joint_label]
        builder.body_label[:] = [s.removeprefix("panda/") for s in builder.body_label]
        self.arm_q_indices, self.arm_dof_indices = [], []
        for index, name in enumerate(JOINT_NAMES):
            joint = builder.joint_label.index(name)
            q, dof = builder.joint_q_start[joint], builder.joint_qd_start[joint]
            self.arm_q_indices.append(q)
            self.arm_dof_indices.append(dof)
            builder.joint_q[q] = HOME[index]
            builder.joint_target_q[q] = HOME[index]
            builder.joint_target_ke[dof] = GAINS[index]
            builder.joint_target_kd[dof] = DAMPING[index]
        self.robot_body_count = len(builder.body_label)
        if builder.body_label != BODY_NAMES:
            raise ValueError(f"Franka visual body contract changed: {builder.body_label}")
        apply_profile(builder, self._contact_profile)
        if self.environment.enabled:
            for index, collider in enumerate(self.environment.colliders):
                builder.add_shape_box(-1,
                                      xform=wp.transform(collider.pose[:3], collider.pose[3:]),
                                      hx=collider.half_extents[0], hy=collider.half_extents[1],
                                      hz=collider.half_extents[2], label=f"room_{index}_{collider.kind}")
        else:
            builder.add_ground_plane()
        self.box_body = None
        if self.has_box:
            self.box_body = builder.add_body(xform=wp.transform((0.8, 0., 0.7), wp.quat_identity()), label="debug_box")
            builder.add_shape_box(self.box_body, hx=0.05, hy=0.05, hz=0.05)
        self.object_bodies = {}
        for obj in self.objects:
            pose = obj.home_pose
            body = builder.add_body(xform=wp.transform(pose[:3], pose[3:]), label=obj.label)
            builder.add_shape_box(body, hx=obj.half_extents[0], hy=obj.half_extents[1], hz=obj.half_extents[2])
            self.object_bodies[obj.id] = body
        self._object_record = self._object_records()
        self.model = builder.finalize(device="cpu")
        solver_type = CpuSolverMuJoCo if self.cpu_cache else newton.solvers.SolverMuJoCo
        self.solver = solver_type(self.model, use_mujoco_cpu=True,
                                               disable_contacts=False, use_mujoco_contacts=True,
                                               integrator="implicitfast", update_data_interval=0,
                                               **self._contact_profile.solver_options())
        self._contact_metadata = native_metadata(self)
        self.control = self.model.control()
        self.state, self.next_state = self.model.state(), self.model.state()
        self.lower = self.model.joint_limit_lower.numpy()[self.arm_dof_indices]
        self.upper = self.model.joint_limit_upper.numpy()[self.arm_dof_indices]
        from .gripper_control import GripperController
        self._gripper_controller = GripperController()
        self._gripper_feedback = None
        self._configure_gripper()
        self.generation += 1
        self.reset()

    def reset(self):
        self._contacts_current = False
        self._gripper_feedback_current = False
        self._gripper_forces = None
        self._gripper_controller.invalidate_feedback()
        self.solver.reset(self.state)
        self.solver.mj_data.time = 0.
        self.solver._mujoco.mj_forward(self.solver.mj_model, self.solver.mj_data)
        self._eval_fk()
        self.control.joint_target_q.assign(self.model.joint_target_q)
        self.control.joint_target_qd.zero_()
        self.step_index, self.sim_time, self.step_cpu_ms = 0, 0., 0.
        self._reset_gripper_control()
        snapshot = self.snapshot()
        self._contacts_current = True  # Existing mj_forward established reset contacts.
        return snapshot

    def configure(self, settings_json):
        settings = _settings(settings_json, self.settings)
        rebuild = any(settings[k] != self.settings[k] for k in ("floating_base", "gravity_scale", "contact_profile"))
        if rebuild:
            self._rebuild(settings, self.has_box)
        else:
            was_legacy = self._gripper_options is None
            self._configure_gripper(settings)
            self.settings = settings
            if was_legacy and self._gripper_options is not None:
                self._reset_gripper_control()
        return self.metadata()

    def _configure_gripper(self, settings=None):
        from .gripper_control import GripperSettings
        from .gripper_feedback import GripperFeedback
        settings = self.settings if settings is None else settings
        speed = settings['gripper_speed_mps']
        options = GripperSettings(settings['gripper_force_hold'], speed, settings['gripper_force_n']) if speed else None
        feedback = self._gripper_feedback
        if options is not None and feedback is None:
            feedback = GripperFeedback(self)
        self._gripper_options, self._gripper_feedback = options, feedback

    def _reset_gripper_control(self):
        # Called AFTER restoration of control arrays on topology/room edits.
        # Reset/replacement owns a fresh controller, never the shallow-copied
        # previous Session's mutable target/acquisition state.
        measured = self.state.joint_q.numpy()[self.arm_q_indices[-2:]]
        target = self.control.joint_target_q.numpy()[self.arm_q_indices[-2:]]
        self._gripper_controller.reset(measured, target)
        self._gripper_controller.invalidate_feedback()
        self._gripper_feedback_current = False
        self._gripper_forces = None

    def gripper_status(self):
        if self._gripper_options is None:
            return 'Legacy position targets; 20 N actuator limit per finger'
        state = self._gripper_controller.diagnostics()
        phase = state['phase'].replace('_', ' ')
        speed = self.settings['gripper_speed_mps'] * 1000.
        force = state['desired_force_n']
        forces = self._gripper_feedback.read() if self._gripper_feedback_current else None
        load = 'unavailable' if forces is None else 'L %.1f / R %.1f N' % forces
        target = f'force target {force:.1f} N/finger' if self._gripper_options.force_hold else 'position target mode'
        return f'{phase}; speed {speed:.0f} mm/s; {target}; last solve load {load}'

    def _eval_fk(self):
        if self.cpu_cache:
            self.solver.eval_fk(self.model, self.state)
        else:
            self.newton.eval_fk(self.model, self.state.joint_q, self.state.joint_qd, self.state)

    def _rebuild(self, settings, has_box):
        # Build transactionally: a malformed asset/dependency failure retains
        # the preceding complete model, settings and generation.
        replacement = object.__new__(Session)
        replacement.__dict__.update(self.__dict__)
        replacement.settings = settings
        replacement.has_box = has_box
        replacement._build()
        self.__dict__.update(replacement.__dict__)

    def set_environment(self, environment_json):
        """Apply one complete room without resetting live motion or controls.

        Static world boxes cannot change the dynamic topology. All construction,
        transfer, contact refresh and snapshot validation complete on a separate
        Session before it replaces the worker's preceding scene.
        """
        environment = parse_environment(environment_json)
        if environment == self.environment:
            return self.snapshot()
        if environment.revision <= self.environment.revision:
            raise ValueError("environment revision must increase")
        replacement = object.__new__(Session)
        replacement.__dict__.update(self.__dict__)
        replacement.environment = environment
        replacement._build()
        replacement._restore_environment_state(self)
        snapshot = replacement.snapshot()
        self.__dict__.update(replacement.__dict__)
        return snapshot

    def _restore_environment_state(self, previous):
        """Transfer only topology-independent state; contact data is rebuilt."""
        old, new = previous.solver, self.solver
        if (self.model.body_label != previous.model.body_label
                or self.model.joint_label != previous.model.joint_label
                or self.box_body != previous.box_body):
            raise ValueError("environment changed the dynamic body/joint ordering")
        # Equal vector sizes alone do not establish the same coordinate mapping.
        for name in ("joint_type", "joint_q_start", "joint_qd_start", "joint_child", "joint_parent"):
            if not self.np.array_equal(getattr(self.model, name).numpy(), getattr(previous.model, name).numpy()):
                raise ValueError(f"environment changed {name}")
        for name in ("mj_q_start", "mj_qd_start", "mjc_body_to_newton"):
            if not self.np.array_equal(getattr(new, name).numpy(), getattr(old, name).numpy()):
                raise ValueError(f"environment changed solver {name}")
        for name in ("nq", "nv", "na", "nu", "nbody", "nmocap", "neq", "nuserdata", "npluginstate"):
            if getattr(new.mj_model, name) != getattr(old.mj_model, name):
                raise ValueError(f"environment changed MuJoCo {name}")
        mj = new._mujoco
        signature = mj.mjtState.mjSTATE_INTEGRATION
        size = mj.mj_stateSize(old.mj_model, signature)
        if size != mj.mj_stateSize(new.mj_model, signature):
            raise ValueError("environment changed MuJoCo integration layout")
        # MuJoCo's native float64 state includes qpos/qvel, activation/history,
        # warmstart, controls/applied forces, equality flags, mocap and time.
        integration = self.np.empty(size, dtype=self.np.float64)
        mj.mj_getState(old.mj_model, old.mj_data, integration, signature)
        mj.mj_setState(new.mj_model, new.mj_data, integration, signature)
        self.state.assign(previous.state)
        self.next_state.assign(previous.next_state)
        self._copy_control_arrays(self.control, previous.control)
        # Fixed-root changes also maintain MJWarp mocap buffers, even though
        # native MjData is authoritative in this CPU solver configuration.
        for name in ("mocap_pos", "mocap_quat"):
            getattr(new.mjw_data, name).assign(getattr(old.mjw_data, name))
        new._step = old._step
        new.mj_model.opt.timestep = old.mj_model.opt.timestep
        self.step_index, self.sim_time, self.step_cpu_ms = previous.step_index, previous.sim_time, previous.step_cpu_ms
        # Contacts, constraint workspaces and derived dynamics belong to the new
        # geometry. Never copy old contact IDs, counts or constraint Jacobians.
        mj.mj_forward(new.mj_model, new.mj_data)
        self._contacts_current = True
        self._reset_gripper_control()

    def _copy_control_arrays(self, destination, source):
        for name in set(vars(destination)) | set(vars(source)):
            target, value = getattr(destination, name, None), getattr(source, name, None)
            if isinstance(target, self.wp.array) or isinstance(value, self.wp.array):
                if (not isinstance(target, self.wp.array) or not isinstance(value, self.wp.array)
                        or target.shape != value.shape or target.dtype != value.dtype):
                    raise ValueError(f"environment changed control array {name}")
                target.assign(value)
            elif (isinstance(target, self.newton.Model.AttributeNamespace)
                  or isinstance(value, self.newton.Model.AttributeNamespace)):
                if target is None or value is None:
                    raise ValueError(f"environment changed control namespace {name}")
                self._copy_control_arrays(target, value)

    def set_base_pose(self, pose):
        if self.settings["floating_base"]:
            raise ValueError("cannot prescribe pose of a dynamic floating base")
        p = self._validated_pose(pose)
        # Prescribing mocap and evaluating Newton FK does not update native
        # MuJoCo contacts. Retire diagnostics, without adding a physics refresh.
        self._contacts_current = False
        self._gripper_feedback_current = False
        self._gripper_forces = None
        self._gripper_controller.invalidate_feedback()
        self.base_pose = p.tolist()
        views = self._cpu_views
        transforms = views.numpy(self.model.joint_X_p) if views is not None else self.model.joint_X_p.numpy()
        transforms[0] = p
        if views is not None:
            views.assign(self.model.joint_X_p, transforms)
        else:
            self.model.joint_X_p.assign(transforms)
        self.solver.notify_model_changed(self.newton.ModelFlags.JOINT_PROPERTIES)
        # Upstream CPU notify does not mirror mocap buffers after JOINT_PROPERTIES.
        mocap_pos, mocap_quat = self.solver.mjw_data.mocap_pos, self.solver.mjw_data.mocap_quat
        self.solver.mj_data.mocap_pos[:] = (views.numpy(mocap_pos) if views is not None else mocap_pos.numpy())[0]
        self.solver.mj_data.mocap_quat[:] = (views.numpy(mocap_quat) if views is not None else mocap_quat.numpy())[0]
        self._eval_fk()
        return self.snapshot()

    def _validated_pose(self, pose):
        p = self.np.asarray(pose, dtype=self.np.float32)
        if p.shape != (7,) or not self.np.isfinite(p).all() or abs(float(self.np.linalg.norm(p[3:])) - 1.) > 1e-3:
            raise ValueError("base pose must contain finite xyz and a unit xyzw quaternion")
        return p

    def step(self, dt: float, targets: list[float], gripper: float, gripper_input_allowed: bool = True) -> bytes:
        np = self.np
        if not math.isfinite(dt) or not 0.0005 <= dt <= 0.02 or abs(dt - self.settings["physics_dt"]) > 1e-8:
            raise ValueError("step dt must match configured physics_dt")
        arm = np.asarray(targets, dtype=np.float32)
        if arm.shape != (7,) or not np.isfinite(arm).all():
            raise ValueError("targets must contain seven finite joint positions")
        if not math.isfinite(gripper) or not 0. <= gripper <= 1.:
            raise ValueError("gripper must be in [0, 1]")
        if type(gripper_input_allowed) is not bool:
            raise ValueError('gripper_input_allowed must be boolean')
        commanded = np.r_[arm, self.lower[7:] + (1. - gripper) * (self.upper[7:] - self.lower[7:])]
        commanded = np.clip(commanded, self.lower, self.upper)
        views = self._cpu_views
        target = views.numpy(self.control.joint_target_q) if views is not None else self.control.joint_target_q.numpy()
        self._gripper_forces = None
        if self._gripper_options is not None:
            measured = (views.numpy(self.state.joint_q) if views is not None else self.state.joint_q.numpy())[self.arm_q_indices[-2:]]
            if self._gripper_options.force_hold and gripper_input_allowed and self._gripper_feedback_current:
                self._gripper_forces = self._gripper_feedback.read()
            commanded[-2:] = self._gripper_controller.step(dt, measured, self._gripper_forces,
                gripper, gripper_input_allowed, self._gripper_options)
        elif not gripper_input_allowed:
            commanded[-2:] = target[self.arm_q_indices[-2:]]
        self._contacts_current = False
        self._gripper_feedback_current = False
        target[self.arm_q_indices] = commanded
        if views is not None:
            views.assign(self.control.joint_target_q, target)
        else:
            self.control.joint_target_q.assign(target)
        start = time.perf_counter()
        self.state.clear_forces()
        self.solver.step(self.state, self.next_state, self.control, None, dt)
        self.state, self.next_state = self.next_state, self.state
        self.step_cpu_ms = (time.perf_counter() - start) * 1000.
        self.step_index += 1
        self.sim_time += dt
        snapshot = self.snapshot()
        self._contacts_current = True
        self._gripper_feedback_current = True
        return snapshot

    def snapshot(self):
        views = self._cpu_views
        poses = (views.numpy(self.state.body_q) if views is not None else self.state.body_q.numpy()).astype("<f4", copy=False)
        if (not self.np.isfinite(poses).all()
                or not self.np.isfinite(views.numpy(self.state.joint_q) if views is not None else self.state.joint_q.numpy()).all()
                or not self.np.isfinite(views.numpy(self.state.joint_qd) if views is not None else self.state.joint_qd.numpy()).all()):
            raise FloatingPointError("Newton returned a non-finite scene")
        objects = self._object_record if views is not None else self._object_records()
        return HEADER.pack(b"QSIM", 1, HEADER.size, len(poses), int(self.has_box) + len(self.objects),
                           self.generation, int(self.solver.mj_data.ncon), self.step_index,
                           self.sim_time, self.step_cpu_ms) + poses.tobytes() + objects

    def _object_records(self):
        legacy = b"" if self.box_body is None else OBJECT.pack(self.box_body, 1, .05, .05, .05)
        return legacy + b"".join(OBJECT.pack(self.object_bodies[obj.id], 1, *obj.half_extents) for obj in self.objects)

    def details_bytes(self, include_contacts=True):
        from .scene_details import details_bytes
        return details_bytes(self, include_contacts)

    def metadata(self):
        versions = dict(newton=self.newton.__version__, warp=self.wp.__version__, numpy=self.np.__version__,
                        mujoco=self.solver._mujoco.__version__, mujoco_warp=self.solver._mujoco_warp.__version__)
        return json.dumps(dict(schema_version=1, backend="Newton SolverMuJoCo CPU", settings=self.settings,
                               body_names=list(self.model.body_label), robot_body_count=self.robot_body_count,
                               joint_names=JOINT_NAMES, initial_q=HOME,
                               lower_limits=self.lower.tolist(), upper_limits=self.upper.tolist(),
                               generation=self.generation, snapshot_header_bytes=HEADER.size,
                               base_pose=self.base_pose, versions=versions,
                               gripper_control=self._gripper_controller.diagnostics() if self._gripper_options is not None else None,
                               contact_model=self._contact_metadata,
                               environment=self.environment.metadata(),
                               user_objects=[dict(id=obj.id, body_index=self.object_bodies[obj.id],
                                                  home_pose=obj.home_pose, half_extents=obj.half_extents) for obj in self.objects],
                               cpu_execution=(dict(self.solver.diagnostics(), views=self._cpu_views.diagnostics()) if self.cpu_cache else
                                              dict(implementation="upstream_newton_cpu"))))

    def joint_positions(self):
        """Raw measured joints in JOINT_NAMES order; no state limit clipping."""
        return self.state.joint_q.numpy()[self.arm_q_indices].tolist()

    def command(self, name):
        if isinstance(name, str) and name.lstrip().startswith("{"):
            from .scene_objects import apply_object_command
            return apply_object_command(self, name)
        if name == "reset":
            return self.reset()
        if name in ("spawn_box", "remove_box"):
            wanted = name == "spawn_box"
            if wanted != self.has_box:
                self._rebuild(self.settings, wanted)
            return self.snapshot()
        raise ValueError(f"unknown scene command: {name}")
