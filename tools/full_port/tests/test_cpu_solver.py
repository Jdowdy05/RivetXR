"""Run with the established pinned Newton/Warp/MuJoCo host environment."""
import sys
import gc
import weakref
from pathlib import Path
import unittest

import numpy as np
import newton
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "quest/app/src/main/python"))
from quest_sim.cpu_solver import CpuSolverMuJoCo
from newton._src.sim.articulation import eval_articulation_fk

URDF = """<robot name="cache_test">
<link name="base"><inertial><mass value="1"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
<link name="arm"><inertial><origin xyz="0 0 -.1"/><mass value="1"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><origin xyz="0 0 -.1"/><geometry><box size=".08 .08 .2"/></geometry></collision></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 -.1"/><axis xyz="0 1 0"/>
<limit lower="-1" upper="1" effort="20" velocity="10"/><dynamics damping=".1" friction=".01"/></joint>
</robot>"""


def make_solver(cls, floating=False, actuated=True):
    builder = newton.ModelBuilder()
    newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
    builder.add_urdf(URDF, floating=floating, xform=wp.transform((0, 0, .6), wp.quat_identity()),
                     enable_self_collisions=True, collapse_fixed_joints=False,
                     force_position_velocity_actuation=actuated)
    builder.joint_target_ke[-1] = 20 if actuated else 0
    builder.joint_target_kd[-1] = 2 if actuated else 0
    if not actuated:
        builder.joint_target_mode[-1] = newton.JointTargetMode.NONE
    builder.add_ground_plane()
    model = builder.finalize(device="cpu")
    solver = cls(model, use_mujoco_cpu=True, use_mujoco_contacts=True,
                 integrator="implicitfast", update_data_interval=0)
    state, next_state, control = model.state(), model.state(), model.control()
    solver.reset(state)
    return model, solver, state, next_state, control


class CpuSolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()
        wp.set_device("cpu")

    def test_fixed_and_floating_exact_step_equivalence_with_forces(self):
        for floating in (False, True):
            baseline = list(make_solver(newton.solvers.SolverMuJoCo, floating))
            cached = list(make_solver(CpuSolverMuJoCo, floating))
            for step in range(120):
                for record in (baseline, cached):
                    model, solver, state, next_state, control = record
                    state.clear_forces()
                    state.body_f.numpy()[-1, 0] = np.float32(.25 if step < 30 else 0)
                    control.joint_f.numpy()[-1] = np.float32(.1 if step < 30 else 0)
                    control.joint_target_q.numpy()[-1] = np.float32(.3 * np.sin(step * .03))
                    solver.step(state, next_state, control, None, .005)
                    record[2], record[3] = next_state, state
                for field in ("joint_q", "joint_qd", "body_q", "body_qd"):
                    np.testing.assert_array_equal(getattr(baseline[2], field).numpy(), getattr(cached[2], field).numpy())
                for field in ("qpos", "qvel", "ctrl", "qfrc_applied", "xfrc_applied"):
                    np.testing.assert_array_equal(getattr(baseline[1].mj_data, field), getattr(cached[1].mj_data, field))
                self.assertEqual(baseline[1].mj_data.ncon, cached[1].mj_data.ncon)
            diagnostics = cached[1].diagnostics()
            self.assertEqual(diagnostics["control_calls"], 120)
            self.assertGreaterEqual(diagnostics["state_calls"], 120)
            self.assertGreater(diagnostics["command_replays"], diagnostics["command_builds"])
            self.assertEqual(diagnostics["buffer_builds"], 1)
            self.assertLessEqual(diagnostics["cached_commands"], 16)

    def test_state_array_replacement_invalidates_commands_and_remains_bounded(self):
        model, solver, state, _, _ = make_solver(CpuSolverMuJoCo)
        solver.eval_fk(model, state)
        for index in range(22):
            state.joint_q = wp.clone(state.joint_q)
            state.joint_q.numpy()[-1] = np.float32(index * .02)
            solver.eval_fk(model, state)
            expected = model.state()
            newton.eval_fk(model, state.joint_q, state.joint_qd, expected)
            np.testing.assert_array_equal(state.body_q.numpy(), expected.body_q.numpy())
        self.assertLessEqual(solver.diagnostics()["cached_commands"], 16)
        self.assertGreaterEqual(solver.diagnostics()["command_builds"], 23)

    def test_access_checks_and_tape_use_upstream_fallback(self):
        model, solver, state, next_state, control = make_solver(CpuSolverMuJoCo)
        previous = wp.config.launch_array_access_mode
        try:
            wp.config.launch_array_access_mode = wp.config.LaunchArrayAccessMode.STRICT
            before = solver.diagnostics()["fallbacks"]
            solver.step(state, next_state, control, None, .005)
            solver.eval_fk(model, next_state)
            self.assertGreaterEqual(solver.diagnostics()["fallbacks"] - before, 3)
        finally:
            wp.config.launch_array_access_mode = previous
        before = solver.diagnostics()["fallbacks"]
        with wp.Tape() as tape:
            solver.eval_fk(model, state)
        self.assertEqual(solver.diagnostics()["fallbacks"], before + 1)
        self.assertTrue(tape.launches)

    def test_zero_actuators_remain_a_valid_noop_launch(self):
        _, solver, state, next_state, control = make_solver(CpuSolverMuJoCo, actuated=False)
        self.assertEqual(solver.mj_model.nu, 0)
        solver.step(state, next_state, control, None, .005)
        self.assertTrue(np.isfinite(next_state.joint_q.numpy()).all())

    def test_command_retains_array_owner_and_module_unload_invalidates(self):
        model, solver, state, _, _ = make_solver(CpuSolverMuJoCo)
        solver.eval_fk(model, state)
        owner = weakref.ref(state.joint_q)
        state.joint_q = wp.clone(state.joint_q)
        gc.collect()
        self.assertIsNotNone(owner(), "cached command lost its array owner")
        solver._cpu_commands.clear()
        gc.collect()
        self.assertIsNone(owner(), "evicted commands retained replaced arrays")
        solver.eval_fk(model, state)
        builds = solver.diagnostics()["command_builds"]
        eval_articulation_fk.module.unload()
        solver.eval_fk(model, state)
        self.assertEqual(solver.diagnostics()["command_builds"], builds + 1)


if __name__ == "__main__":
    unittest.main()
