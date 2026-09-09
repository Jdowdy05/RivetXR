# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
# Adapted argument ordering and CPU synchronization from the pinned solver.
"""Allocation/dispatch caching for the pinned Newton MuJoCo CPU adapter.

Audited against Newton d37f4d3d341ccce1e06a1dff21e9a054759b4855 and Warp
d4de134b97b961f1a19bd76830e71ea7f9df2470. This intentionally uses those pinned
private kernel bindings, but only public Warp Launch commands. Solver.step,
MuJoCo integration, numerical kernels, force handling and Newton FK are intact.
Caches are solver-owned and bounded; no global runtime/module is modified.
"""
from collections import OrderedDict

import numpy as np
import newton
import warp as wp
from warp._src import context as _wp_context
from newton._src.sim.articulation import eval_articulation_fk
from newton._src.solvers.mujoco import kernels


class CpuSolverMuJoCo(newton.solvers.SolverMuJoCo):
    """Pinned CPU fast path with unchanged inherited step/reset/model updates."""

    _MAX_COMMANDS = 16

    def __init__(self, *args, **kwargs):
        # The base constructor can invoke overridable synchronization methods.
        self._cpu_commands = OrderedDict()
        self._cpu_buffer_key = None
        self._cpu_buffers = None
        self._cpu_counts = dict(control_calls=0, state_calls=0, fk_calls=0,
                                command_builds=0, command_replays=0, fallbacks=0,
                                buffer_builds=0)
        super().__init__(*args, **kwargs)

    def diagnostics(self):
        return dict(self._cpu_counts, implementation="pinned_newton_cpu_cached_v1",
                    cached_commands=len(self._cpu_commands),
                    max_cached_commands=self._MAX_COMMANDS,
                    buffer_sets=int(self._cpu_buffers is not None))

    def _allowed(self, model):
        runtime = _wp_context.runtime
        return (getattr(self, "use_mujoco_cpu", False) and model is self.model
                and model.device.is_cpu and not model.requires_grad
                and model.world_count == 1
                and not self._total_loop_joint_dofs and not self._total_loop_joint_coords
                and runtime is not None and runtime.tape is None
                and getattr(runtime, "_apic_capture", None) is None
                and not runtime.captures
                and wp.config.launch_array_access_mode == wp.config.LaunchArrayAccessMode.RELAXED
                and not wp.config.verify_autograd_array_access
                and not wp.config.verify_fp and not wp.config.verify_cuda
                and not wp.config.print_launches)

    def _fallback(self):
        self._cpu_counts["fallbacks"] += 1

    def _buffers_for(self, model, data):
        # No MuJoCo NumPy views are retained. Current MuJoCo buffers are read or
        # assigned on each call, preserving the float64 integration storage.
        key = (id(model), id(data), id(model.device), len(data.ctrl),
               len(data.qfrc_applied), len(data.xfrc_applied), len(data.qpos), len(data.qvel))
        if key != self._cpu_buffer_key:
            self._cpu_commands.clear()
            ctrl = wp.zeros((1, len(data.ctrl)), dtype=wp.float32, device=model.device)
            qfrc = wp.zeros((1, len(data.qfrc_applied)), dtype=wp.float32, device=model.device)
            xfrc = wp.zeros((1, len(data.xfrc_applied)), dtype=wp.spatial_vector, device=model.device)
            qpos = wp.empty((1, len(data.qpos)), dtype=wp.float32, device=model.device)
            qvel = wp.empty((1, len(data.qvel)), dtype=wp.float32, device=model.device)
            self._cpu_buffers = (ctrl, qfrc, xfrc, qpos, qvel,
                                 ctrl.numpy(), qfrc.numpy(), xfrc.numpy(), qpos.numpy(), qvel.numpy())
            self._cpu_buffer_key = key
            self._cpu_counts["buffer_builds"] += 1
        return self._cpu_buffers

    @staticmethod
    def _argument_key(value):
        if isinstance(value, wp.array):
            if not value.device.is_cpu or value.requires_grad or not value.is_contiguous:
                return None
            return (id(value), value.ptr, value.shape, value.strides, value.dtype, id(value.device))
        if value is None or isinstance(value, (bool, int, float)):
            return (type(value), value)
        return None

    def _run(self, name, kernel, dim, inputs, outputs, device):
        if dim == 0 or (isinstance(dim, tuple) and 0 in dim):
            # Warp's zero-sized launch is a no-op and record_cmd returns None.
            # Keep that behavior for actuator-free models (nu == 0).
            wp.launch(kernel, dim=dim, inputs=inputs, outputs=outputs, device=device)
            return
        args = inputs + outputs
        bindings = tuple(self._argument_key(arg) for arg in args)
        if any(binding is None for binding in bindings):
            # Nonstandard arguments still use Warp's ordinary checking/packing.
            self._fallback()
            wp.launch(kernel, dim=dim, inputs=inputs, outputs=outputs, device=device)
            return
        # load() respects module invalidation and retains the currently selected
        # executable. A changed module executable cannot reuse an old command.
        module_exec = kernel.module.load(device, 1)
        key = (name, id(kernel), id(module_exec), dim, bindings)
        command = self._cpu_commands.get(key)
        if command is None:
            command = wp.launch(kernel, dim=dim, inputs=inputs, outputs=outputs,
                                device=device, record_cmd=True)
            if command is None:
                raise RuntimeError("Pinned Warp did not return a CPU launch command")
            # In the pinned Warp version the public command retains fwd_args
            # (including every array owner) and module_exec, not bare pointers.
            if command.module_exec is not module_exec:
                raise RuntimeError("Warp CPU module changed while recording its launch")
            self._cpu_commands[key] = command
            self._cpu_counts["command_builds"] += 1
            if len(self._cpu_commands) > self._MAX_COMMANDS:
                self._cpu_commands.popitem(last=False)
        else:
            self._cpu_commands.move_to_end(key)
        command.launch()
        self._cpu_counts["command_replays"] += 1

    def _apply_mjc_control(self, model, state, control, mj_data):
        if (not self._allowed(model) or self._data_is_mjwarp(mj_data)
                or control is None or control.joint_f is None or state.body_f is None
                or self.mjc_actuator_ctrl_source is None or self.mjc_actuator_to_newton_idx is None
                or len(mj_data.qfrc_applied) != model.joint_dof_count):
            self._fallback()
            return super()._apply_mjc_control(model, state, control, mj_data)
        self._cpu_counts["control_calls"] += 1
        ctrl, qfrc, xfrc, _, _, ctrl_np, qfrc_np, xfrc_np, _, _ = self._buffers_for(model, mj_data)
        # Keep every initialization performed by upstream wp.zeros. In
        # particular, the free-joint wrench kernel accumulates into xfrc.
        ctrl.zero_(); qfrc.zero_(); xfrc.zero_()
        joints_per_world = model.joint_count
        nu = self.mjc_actuator_ctrl_source.shape[0]
        target_q_per_world = control.joint_target_q.shape[0] if control.joint_target_q is not None else 0
        namespace = getattr(control, "mujoco", None)
        mujoco_ctrl = getattr(namespace, "ctrl", None) if namespace is not None else None
        ctrls_per_world = mujoco_ctrl.shape[0] if mujoco_ctrl is not None else 0
        self._run("control", kernels.apply_mjc_control_kernel, (1, nu), [
            self.mjc_actuator_ctrl_source, self.mjc_actuator_to_newton_idx,
            self.mjc_actuator_to_newton_target_q_idx, self.mjc_actuator_to_target_q_axis_idx,
            self.mjc_actuator_to_newton_ball_jnt, model.joint_X_c,
            control.joint_target_q, control.joint_target_qd, state.joint_q, mujoco_ctrl,
            target_q_per_world, model.joint_coord_count, model.joint_dof_count,
            ctrls_per_world, joints_per_world, model.use_coord_layout_targets,
        ], [ctrl], model.device)
        self._run("joint_force", kernels.apply_mjc_qfrc_kernel, (1, joints_per_world), [
            control.joint_f, state.joint_q, model.joint_type, model.joint_child,
            model.body_flags, model.joint_q_start, model.joint_qd_start,
            model.joint_dof_dim, model.joint_X_c, joints_per_world, self.mj_qd_start,
        ], [qfrc], model.device)
        nbody = self.mjc_body_to_newton.shape[1]
        self._run("body_force", kernels.apply_mjc_body_f_kernel, (1, nbody), [
            self.mjc_body_to_newton, model.body_flags, state.body_f, model.body_mass,
            model.body_world, model.gravity, self.mjw_model.body_gravcomp,
        ], [xfrc], model.device)
        self._run("free_joint_force", kernels.apply_mjc_free_joint_f_to_body_f_kernel, (1, nbody), [
            self.mjc_body_to_newton, model.body_flags, self.body_free_qd_start, control.joint_f,
        ], [xfrc], model.device)
        mj_data.xfrc_applied = xfrc_np
        mj_data.ctrl[:] = ctrl_np.reshape(-1)
        mj_data.qfrc_applied[:] = qfrc_np

    def _update_newton_state(self, model, state, mj_data, state_prev):
        qfrc_actuator = getattr(getattr(state, "mujoco", None), "qfrc_actuator", None)
        if (not self._allowed(model) or self._data_is_mjwarp(mj_data)
                or state.body_qdd is not None or state.body_parent_f is not None
                or qfrc_actuator is not None or len(mj_data.qpos) != model.joint_coord_count):
            self._fallback()
            return super()._update_newton_state(model, state, mj_data, state_prev)
        self._cpu_counts["state_calls"] += 1
        _, _, _, qpos, qvel, _, _, _, qpos_np, qvel_np = self._buffers_for(model, mj_data)
        # These casts reproduce wp.array([mj_data.qpos/qvel], float32). Never
        # alias or reinterpret MuJoCo's double precision integration arrays.
        np.copyto(qpos_np[0], mj_data.qpos, casting="unsafe")
        np.copyto(qvel_np[0], mj_data.qvel, casting="unsafe")
        qpos.mark_init(); qvel.mark_init()
        namespace = getattr(model, "mujoco", None)
        dof_ref = getattr(namespace, "dof_ref", None) if namespace is not None else None
        self._run("coordinates", kernels.convert_mj_coords_to_warp_kernel, (1, model.joint_count), [
            qpos, qvel, model.joint_count, model.joint_type, model.joint_q_start,
            model.joint_qd_start, model.joint_dof_dim, model.joint_child,
            model.joint_X_p, model.joint_X_c, model.body_com, dof_ref, model.body_flags,
            state_prev.joint_q, state_prev.joint_qd, self.mj_q_start, self.mj_qd_start,
        ], [state.joint_q, state.joint_qd], model.device)
        self.eval_fk(model, state)

    def eval_fk(self, model, state):
        """Same default Newton CPU FK, reusable by Session reset/base updates."""
        if not self._allowed(model):
            self._fallback()
            return newton.eval_fk(model, state.joint_q, state.joint_qd, state)
        if model.articulation_count == 0:
            return
        self._cpu_counts["fk_calls"] += 1
        self._run("fk", eval_articulation_fk, model.articulation_count, [
            model.articulation_start, model.articulation_end, model.articulation_count,
            None, None, model.joint_articulation, state.joint_q, state.joint_qd,
            model.joint_q_start, model.joint_qd_start, model.joint_type,
            model.joint_parent, model.joint_child, model.joint_X_p, model.joint_X_c,
            model.joint_axis, model.joint_dof_dim, model.body_com, model.body_flags,
            newton.BodyFlags.ALL,
        ], [state.body_q, state.body_qd], model.device)
