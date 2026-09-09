"""Host-only Franka capture. No robot or headset connection is opened."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from tools.newton_codegen.artifact_manifest import (
    NEWTON_COMMIT, NEWTON_VERSION, WARP_COMMIT, WARP_VERSION, GRAPH_OPERATIONS,
    canonical_json, canonicalize_graph,
)

ARM_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
JOINT_NAMES = ARM_NAMES + ['panda_finger_joint1', 'panda_finger_joint2']


def collect_module_sources(modules, destination):
    """Preserve module identity, including when cache files share a basename."""
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    collected = {}
    for module_hash, info in modules:
        identity = canonical_json([info['module_name'], module_hash])
        candidates = list(Path(info['binary_path']).parent.glob('*.cpp'))
        if len(candidates) != 1:
            raise ValueError(f'Expected one CPU source for module {identity.strip()}')
        content = candidates[0].read_bytes()
        name = hashlib.sha256(identity.encode()).hexdigest() + '.cpp'
        if name in collected and collected[name] != (identity, content):
            raise ValueError('Conflicting CPU source for module identity')
        target = destination / name
        if target.exists() and target.read_bytes() != content:
            raise ValueError('CPU source output collision')
        target.write_bytes(content)
        collected[name] = (identity, content)
    if not collected:
        raise ValueError('No captured CPU modules')
    return sorted(collected)


def validate_state(q, qd, count):
    if len(q) != count or len(qd) != count or not all(math.isfinite(v) for v in [*q, *qd]):
        raise ValueError('State requires exact counts and finite q/qd')


def validate_config(config):
    if config['joint_names'] != JOINT_NAMES:
        raise ValueError('Expected seven ordered Panda arm joints followed by two fingers')
    validate_state(config['initial_q'], config['initial_qd'], 9)
    for key in ('lower_limits', 'upper_limits', 'stiffness', 'damping'):
        if len(config[key]) != 9 or not all(math.isfinite(v) for v in config[key]):
            raise ValueError(f'Invalid {key}')
    for low, high, home in zip(config['lower_limits'], config['upper_limits'], config['initial_q']):
        if not low < high or not low <= home <= high:
            raise ValueError('Invalid joint limits or home pose')
    if config['timestep'] != 0.001 or config['gravity'] != [0, 0, 0]:
        raise ValueError('Capture requires 1 ms timestep and disabled gravity')
    if config['stiffness'] != [400] * 7 + [2000] * 2 or config['damping'] != [80] * 7 + [100] * 2:
        raise ValueError('Capture requires the specified high-PD gains')
    if config['effort_limits'] != [87] * 4 + [12] * 3 + [200] * 2:
        raise ValueError('Capture requires the Isaac Lab high-PD effort limits')


def load_config(path=None):
    path = Path(path) if path else Path(__file__).resolve().parents[2] / 'config/franka_panda.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    validate_config(config)
    return config


def build_model(description_root, config):
    import newton
    validate_config(config)
    source = Path(description_root) / 'robots/panda_arm_hand.urdf'
    if not source.is_file():
        raise ValueError(f'Required Franka URDF missing: {source}')
    tree = ET.parse(source)
    # Geometry is a separate rendering artifact. Retain every link, joint,
    # inertial frame, mass and tensor, including both independent finger DOFs.
    for link in tree.getroot().findall('link'):
        for element in list(link):
            if element.tag in ('visual', 'collision'):
                link.remove(element)
    builder = newton.ModelBuilder(gravity=(0.0, 0.0, 0.0))
    builder.add_urdf(ET.tostring(tree.getroot(), encoding='unicode'), floating=False,
                     enable_self_collisions=False, collapse_fixed_joints=False,
                     force_position_velocity_actuation=True)
    builder.joint_label[:] = [name.removeprefix('panda/') for name in builder.joint_label]
    builder.body_label[:] = [name.removeprefix('panda/') for name in builder.body_label]
    movable = [name for name, kind in zip(builder.joint_label, builder.joint_type)
               if kind != newton.JointType.FIXED]
    if movable != config['joint_names'] or builder.joint_dof_count != 9:
        raise ValueError(f'Unexpected URDF joint order: {movable}')
    for actual, expected in [(builder.joint_limit_lower, config['lower_limits']),
                             (builder.joint_limit_upper, config['upper_limits'])]:
        if len(actual) != 9 or any(abs(a - b) > 1e-6 for a, b in zip(actual, expected)):
            raise ValueError('URDF joint limits differ from capture contract')
    builder.joint_q[:] = config['initial_q']
    builder.joint_qd[:] = config['initial_qd']
    builder.joint_target_ke[:] = config['stiffness']
    builder.joint_target_kd[:] = config['damping']
    builder.joint_target_q[:] = config['initial_q']
    builder.joint_target_qd[:] = config['initial_qd']
    return builder.finalize(device='cpu')


def build_actuator(model, config):
    import warp as wp
    from newton.actuators import Actuator, ClampingMaxEffort, DrivePD, ResponseOracle
    validate_config(config)
    # Featherstone's legacy target arrays use explicit PD. With Panda wrist
    # inertias, the specified high gains are unstable at 1 ms. Move the exact
    # same gains to Newton's coupled implicit drive and disable the duplicate
    # legacy force contribution; no inertias or gains are altered.
    oracle = ResponseOracle(model)
    actuator = Actuator(indices=wp.array(list(range(9)), dtype=wp.uint32, device='cpu'),
                        drive=DrivePD(kp=wp.clone(model.joint_target_ke),
                                      kd=wp.clone(model.joint_target_kd)),
                        control_feedforward_attr=None)
    actuator.set_effort_mode_implicit(response=oracle)
    effort_clamp = ClampingMaxEffort(max_effort=wp.array(config['effort_limits'],
                                                       dtype=wp.float32, device='cpu'))
    model.joint_target_ke.zero_()
    model.joint_target_kd.zero_()
    return oracle, actuator, effort_clamp


def step_actuator(drive, state, control, timestep):
    oracle, actuator, effort_clamp = drive
    control.joint_f.zero_()
    oracle.refresh(state)
    actuator.step(state, control, dt=timestep)
    # Clamping inside Newton's nonsmooth implicit residual caused a wrist
    # limit cycle with either supported warm-start mode. Saturate the solved
    # implicit drive instead, before adding independent external forces.
    effort_clamp.modify_forces(control.joint_f, control.joint_f,
                               state.joint_q, state.joint_qd,
                               actuator.pos_indices, actuator.indices, device='cpu')


def capture(description_root, output, cache, config=None):
    import newton
    import warp as wp
    if (newton.__version__, wp.__version__) != (NEWTON_VERSION, WARP_VERSION):
        raise ValueError('Pinned Newton/Warp versions required; set source PYTHONPATH')
    config = config or load_config()
    validate_config(config)
    output, cache = Path(output), Path(cache)
    output.mkdir(parents=True, exist_ok=True)
    wp.config.kernel_cache_dir = str(cache.resolve())
    wp.config.enable_backward = False
    wp.init()
    model = build_model(description_root, config)
    state_in, state_out = model.state(), model.state()
    control = model.control()
    drive = build_actuator(model, config)
    from tools.newton_codegen.kernels import add_external_force
    external_force = wp.zeros(9, dtype=wp.float32, device='cpu')
    solver = newton.solvers.SolverFeatherstone(model)

    def step():
        state_in.clear_forces()
        step_actuator(drive, state_in, control, config['timestep'])
        wp.launch(add_external_force, dim=9, inputs=[external_force, control.joint_f], device='cpu')
        solver.step(state_in, state_out, control, None, config['timestep'])

    newton.eval_fk(model, state_in.joint_q, state_in.joint_qd, state_in)
    step()
    wp.synchronize_device('cpu')
    graphs = []
    for kind in ('reset', 'step'):
        state_in, state_out = model.state(), model.state()
        newton.eval_fk(model, state_in.joint_q, state_in.joint_qd, state_in)
        wp.capture_begin(device='cpu', apic=True, force_module_load=False)
        if kind == 'reset':
            wp.copy(state_out.joint_q, state_in.joint_q)
            wp.copy(state_out.joint_qd, state_in.joint_qd)
            newton.eval_fk(model, state_out.joint_q, state_out.joint_qd, state_out)
        else:
            step()
        graph = wp.capture_end(device='cpu')
        inputs = {'joint_q_in': state_in.joint_q, 'joint_qd_in': state_in.joint_qd}
        if kind == 'step':
            inputs.update(joint_force=external_force, joint_target_q=control.joint_target_q,
                          joint_target_qd=control.joint_target_qd)
        outputs = {'joint_q_out': state_out.joint_q, 'joint_qd_out': state_out.joint_qd,
                   'body_q_out': state_out.body_q}
        wp.capture_save(graph, str(output / f'franka_{kind}'), inputs=inputs, outputs=outputs)
        graph_file = output / f'franka_{kind}.wrp'
        graph_file.write_bytes(canonicalize_graph(graph_file.read_bytes(),
                                                 expected_operations=GRAPH_OPERATIONS[kind]))
        graphs.append(graph)
    sources = collect_module_sources(
        [item for graph in graphs for item in graph._apic_capture.collected_modules.items()],
        output / 'generated_cpp')
    required_symbols = sorted({info[key] for graph in graphs
                               for info in graph._apic_capture.collected_kernels.values()
                               for key in ('forward_name', 'backward_name') if info.get(key)})
    metadata = dict(config, schema_version=1, newton_commit=NEWTON_COMMIT,
                    warp_commit=WARP_COMMIT, newton_version=NEWTON_VERSION,
                    warp_version=WARP_VERSION, joint_count=9, body_count=model.body_count,
                    body_names=list(model.body_label), generated_cpp=sorted(sources),
                    required_kernel_symbols=required_symbols,
                    joint_units=['rad'] * 7 + ['m'] * 2,
                    body_transform_layout='px,py,pz,qx,qy,qz,qw',
                    body_frame='robot_base', collisions=False,
                    drive_mode='newton_actuator_coupled_implicit_pd', internal_substeps=1,
                    effort_limit_source='IsaacLab FRANKA_PANDA_HIGH_PD_CFG effort_limit_sim',
                    effort_clamp_mode='post_implicit_solve_before_external_force',
                    velocity_limits_enforced=False,
                    target_limits_enforced_by='downstream_runtime_ik',
                    urdf_sha256=hashlib.sha256((Path(description_root) / 'robots/panda_arm_hand.urdf').read_bytes()).hexdigest(),
                    step_buffers={name: model.body_count * 28 if name == 'body_q_out' else 36
                                  for name in [*inputs, *outputs]},
                    reset_buffers={name: model.body_count * 28 if name == 'body_q_out' else 36
                                   for name in ('joint_q_in', 'joint_qd_in', 'joint_q_out', 'joint_qd_out', 'body_q_out')})
    (output / 'capture_metadata.json').write_text(canonical_json(metadata), encoding='utf-8')
    print(f'Captured nine joints, {model.body_count} bodies, {len(sources)} CPU modules')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--franka-description-root', default=os.environ.get('FRANKA_DESCRIPTION_ROOT'))
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cache', required=True, type=Path)
    args = parser.parse_args()
    if not args.franka_description_root:
        parser.error('Set FRANKA_DESCRIPTION_ROOT or --franka-description-root')
    capture(args.franka_description_root, args.output, args.cache)


if __name__ == '__main__':
    main()
