"""Generate expected states using production C++ input mapping and real APIC Newton.

Run with the pinned host Python and PYTHONPATH pointing at the pinned Warp and
Newton source checkouts. No simplified dynamics or injected expected states are
used. A failed substep never publishes a partial expected-state artifact.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

from tools.newton_codegen.artifact_manifest import canonical_json, verify_artifacts
from tools.verification.generate_controller_trace import finite_vector, validate_trace
from tools.verification.state_limits import (
    ALLOWANCES, STATE_LIMIT_POLICY, StateLimitStats, float32, measure_excursions,
    validate_policy, validate_statistics,
)


def validate_reference(reference, trace_hash, manifest_hash, manifest):
    validate_policy(reference.get('state_limit_policy'))
    if (reference.get('schema_version') != 1 or reference.get('trace_sha256') != trace_hash or
            reference.get('physics_manifest_sha256') != manifest_hash):
        raise ValueError('C++ reference source identity differs')
    samples = reference.get('samples')
    if not isinstance(samples, list) or len(samples) != 1000:
        raise ValueError('C++ reference must contain exactly 1000 samples')
    previous = manifest['initial_q'][:7]
    low, high = previous[:], previous[:]
    for index, sample in enumerate(samples):
        # Literal independent acceptance script, not copied from golden state or
        # inferred from mapper output: held reacquisition is always disengaged.
        engaged = (30 <= index < 350 or 380 <= index < 400 or 440 <= index < 600 or
                   640 <= index < 850 or 900 <= index < 980)
        if (type(sample.get('index')) is not int or sample['index'] != index or
                type(sample.get('engaged')) is not bool or sample['engaged'] != engaged or
                type(sample.get('calibrated')) is not bool or sample['calibrated'] != (index >= 1)):
            raise ValueError(f'C++ reference control semantics differ at sample {index}')
        targets = sample.get('targets')
        finite_vector(targets, 7, 'targets')
        if any(not lower <= q <= upper for q, lower, upper in
               zip(targets, map(float32, manifest['lower_limits']), map(float32, manifest['upper_limits']))):
            raise ValueError(f'C++ target outside limits at sample {index}')
        if any(abs(q-p) > .040001 for q, p in zip(targets, previous)):
            raise ValueError(f'C++ target delta exceeds 0.04 rad at sample {index}')
        # At sample zero account only for the textual double versus C++ float
        # representation of the declared home. Later holds must be bit-exact.
        if not engaged and any(abs(q-p) > (2e-7 if index == 0 else 0.) for q, p in zip(targets, previous)):
            raise ValueError(f'C++ disengaged target changed at sample {index}')
        low = [min(a, b) for a, b in zip(low, targets)]
        high = [max(a, b) for a, b in zip(high, targets)]
        previous = targets
    if any(b-a <= .005 for a, b in zip(low, high)):
        raise ValueError('C++ trajectory must exercise every arm joint by more than 0.005 rad')


def validate_physics_state(q, qd, bodies, lower, upper, substep):
    finite_vector(q, 9, 'joint_q')
    finite_vector(qd, 9, 'joint_qd')
    finite_vector(bodies, 84, 'body_q')
    excursions = measure_excursions(q, lower, upper)
    for joint, (excess, allowance) in enumerate(zip(excursions, ALLOWANCES)):
        if excess > allowance:
            raise ValueError(f'Real Newton state allowance violation at substep {substep}, '
                             f'joint {joint}: raw excursion {excess} exceeds {allowance}')
    return excursions


def verify_host_sources(newton, wp, manifest):
    for module, name in [(newton, 'newton'), (wp, 'warp')]:
        if module.__version__ != manifest[f'{name}_version']:
            raise ValueError(f'Pinned {name} version required')
        commit = subprocess.check_output(['git', '-C', str(Path(module.__file__).parent),
                                          'rev-parse', 'HEAD'], text=True).strip()
        if commit != manifest[f'{name}_commit']:
            raise ValueError(f'Pinned {name} source commit required')


def generate(artifact_dir, description_root, trace_path, reference_executable, output):
    artifact_dir, description_root = Path(artifact_dir), Path(description_root)
    manifest_bytes = (artifact_dir / 'artifact_manifest.json').read_bytes()
    manifest, trace_bytes = json.loads(manifest_bytes), Path(trace_path).read_bytes()
    validate_trace(json.loads(trace_bytes), manifest)
    verify_artifacts(manifest, artifact_dir)
    urdf = description_root / 'robots/panda_arm_hand.urdf'
    if hashlib.sha256(urdf.read_bytes()).hexdigest() != manifest['urdf_sha256']:
        raise ValueError('Source URDF hash differs from captured physics')
    trace_hash, manifest_hash = hashlib.sha256(trace_bytes).hexdigest(), hashlib.sha256(manifest_bytes).hexdigest()
    with tempfile.TemporaryDirectory(prefix='newton-trace-reference-') as directory:
        mapped_path = Path(directory) / 'mapped.json'
        subprocess.run([str(Path(reference_executable).resolve()), str(mapped_path)], check=True)
        reference = json.loads(mapped_path.read_bytes())
    validate_reference(reference, trace_hash, manifest_hash, manifest)

    import newton
    import warp as wp
    verify_host_sources(newton, wp, manifest)
    wp.config.kernel_cache_dir = str(artifact_dir / '_build/trace-reference-cache')
    wp.init()
    graph = wp.capture_load(str(artifact_dir / 'franka_step.wrp'), device='cpu')
    expected_buffers = {'joint_q_in': 36, 'joint_qd_in': 36, 'joint_q_out': 36,
                        'joint_qd_out': 36, 'body_q_out': 336, 'joint_force': 36,
                        'joint_target_q': 36, 'joint_target_qd': 36}
    if {name: info['size'] for name, info in graph._params.items()} != expected_buffers:
        raise ValueError('Real step graph external buffer contract differs')
    q = wp.array(manifest['initial_q'], dtype=wp.float32, device='cpu')
    qd = wp.zeros(9, dtype=wp.float32, device='cpu')
    zero = wp.zeros(9, dtype=wp.float32, device='cpu')
    target = wp.array(manifest['initial_q'], dtype=wp.float32, device='cpu')
    bodies = wp.zeros(12, dtype=wp.transform, device='cpu')
    # Use the same float32 manifest bounds as native verification; preserve and
    # report raw excursions on every substep without changing physics or targets.
    lower, upper = [float32(v) for v in manifest['lower_limits']], [float32(v) for v in manifest['upper_limits']]
    graph.set_param('joint_target_qd', zero)
    graph.set_param('joint_force', zero)
    q_min, q_max = manifest['initial_q'][:], manifest['initial_q'][:]
    qd_max, step = 0., 0
    state_limits = StateLimitStats()
    for sample in reference['samples']:
        target.assign(sample['targets'] + [.04, .04])
        graph.set_param('joint_target_q', target)
        for _ in range(10):
            graph.set_param('joint_q_in', q)
            graph.set_param('joint_qd_in', qd)
            wp.capture_launch(graph)
            graph.get_param('joint_q_out', q)
            graph.get_param('joint_qd_out', qd)
            graph.get_param('body_q_out', bodies)
            step += 1
            q_values, qd_values = q.numpy().tolist(), qd.numpy().tolist()
            body_values = bodies.numpy().reshape(-1).tolist()
            state_limits.observe(q_values, lower, upper)
            validate_physics_state(q_values, qd_values, body_values, lower, upper, step)
            q_min = [min(a, b) for a, b in zip(q_min, q_values)]
            q_max = [max(a, b) for a, b in zip(q_max, q_values)]
            qd_max = max(qd_max, *(abs(v) for v in qd_values))
        sample['q'], sample['qd'] = q_values, qd_values
        if (sample['index'] + 1) % 100 == 0:
            print(f'Real Newton: {sample["index"] + 1}/1000 samples, {step}/10000 checked substeps', flush=True)
    if step != 10000:
        raise ValueError('Incomplete real physics reference')
    validate_statistics(state_limits.as_dict(), step)
    # Re-check artifact identities after replay before publishing tracked data.
    verify_artifacts(manifest, artifact_dir)
    if (artifact_dir / 'artifact_manifest.json').read_bytes() != manifest_bytes:
        raise ValueError('Physics manifest changed during replay')
    reference.update(tolerances={'joint_q': 1e-5, 'joint_qd': 1e-4}, physics_substeps=step,
                     substeps_per_sample=10, timestep_seconds=.001,
                     state_limit_policy=dict(STATE_LIMIT_POLICY), state_limits=state_limits.as_dict(),
                     measured={'q_min': q_min, 'q_max': q_max, 'max_abs_qd': qd_max})
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + '.tmp')
    temporary.write_text(canonical_json(reference), encoding='utf-8', newline='\n')
    temporary.replace(output)
    print(f'Published real 10000-substep reference; max |qd|={qd_max:.9g}; '
          f'state_limits={json.dumps(state_limits.as_dict(), sort_keys=True)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    parser.add_argument('--franka-description-root', type=Path, required=True)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--reference-executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.artifact_dir, args.franka_description_root, args.trace, args.reference_executable, args.output)


if __name__ == '__main__':
    main()
