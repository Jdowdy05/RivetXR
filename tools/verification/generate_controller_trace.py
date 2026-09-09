"""Validate a committed URDF-derived control trace and emit its native header.

Ordinary builds never author new input. --create-source explicitly creates the
canonical deterministic fixture using independent stdlib URDF FK.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from tools.newton_codegen.artifact_manifest import (
    NEWTON_COMMIT, NEWTON_VERSION, WARP_COMMIT, WARP_VERSION, canonical_json,
)
from tools.newton_codegen.capture_franka import load_config

URDF_SHA256 = '5aece94c95106e722825bc0487ec9ba3b1162a202c00f38a936a7a18523e8c92'
SAMPLE_COUNT = 1000
SUBSTEPS = 10


def contract_manifest():
    """Required production pins, shared with the existing capture configuration."""
    return dict(load_config(), schema_version=1, joint_count=9, body_count=12,
                body_names=[f'panda_link{i}' for i in range(9)] +
                ['panda_hand', 'panda_leftfinger', 'panda_rightfinger'],
                urdf_sha256=URDF_SHA256, newton_commit=NEWTON_COMMIT,
                newton_version=NEWTON_VERSION, warp_commit=WARP_COMMIT,
                warp_version=WARP_VERSION)


def phase_input(index):
    """Canonical operator script: all other activity/validity flags stay true."""
    if index == 0:
        return 'initial_release', 0., 511
    if index == 1:
        return 'calibrate', 0., 1023
    if index < 30:
        return 'calibrated_release', 0., 511
    if index < 350:
        return 'engaged_motion', .8, 511
    if index < 380:
        return 'released_moving_grip', 0., 511
    if index < 400:
        return 'reengaged_motion', .8, 511
    if index < 410:
        return 'position_tracking_loss', .8, 383
    if index < 430:
        return 'tracking_restored_held', .8, 511
    if index < 440:
        return 'tracking_rearm_release', 0., 511
    if index < 600:
        return 'tracking_rearmed_motion', .8, 511
    if index < 610:
        return 'focus_loss', .8, 510
    if index < 630:
        return 'focus_restored_held', .8, 511
    if index < 640:
        return 'focus_rearm_release', 0., 511
    if index < 850:
        return 'focus_rearmed_motion', .8, 511
    if index < 870:
        return 'pose_inactive', .8, 507
    if index < 890:
        return 'pose_restored_held', .8, 511
    if index < 900:
        return 'pose_rearm_release', 0., 511
    if index < 980:
        return 'pose_rearmed_motion', .8, 511
    return 'final_release', 0., 511


def finite_vector(values, count, name):
    if (not isinstance(values, list) or len(values) != count or
            any(type(v) not in (int, float) or not math.isfinite(v) or
                abs(v) > 3.4028234663852886e38 for v in values)):
        raise ValueError(f'{name} must contain {count} finite float32 numbers')


def validate_trace(data, manifest):
    for key, expected in contract_manifest().items():
        if manifest.get(key) != expected:
            raise ValueError(f'Physics contract pin differs: {key}')
    required = {'schema_version', 'sample_count', 'timestep_seconds',
                'substeps_per_sample', 'urdf_sha256', 'samples'}
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError('Trace schema fields differ')
    for key, expected in [('schema_version', 1), ('sample_count', SAMPLE_COUNT),
                          ('timestep_seconds', .001), ('substeps_per_sample', SUBSTEPS),
                          ('urdf_sha256', manifest['urdf_sha256'])]:
        if type(data[key]) is not type(expected) or data[key] != expected:
            raise ValueError(f'Trace contract differs: {key}')
    if not isinstance(data['samples'], list) or len(data['samples']) != SAMPLE_COUNT:
        raise ValueError('Trace requires exactly 1000 samples')
    for index, sample in enumerate(data['samples']):
        if not isinstance(sample, dict) or set(sample) != {'index', 'position', 'rotation', 'trigger', 'flags', 'phase'}:
            raise ValueError(f'Sample {index} schema fields differ')
        if type(sample['index']) is not int or sample['index'] != index:
            raise ValueError(f'Sample {index} index differs')
        finite_vector(sample['position'], 3, 'position')
        finite_vector(sample['rotation'], 4, 'rotation XYZW')
        if abs(sum(v * v for v in sample['rotation']) - 1.) > 1e-6:
            raise ValueError(f'Sample {index} quaternion must be normalized XYZW')
        trigger, flags = sample['trigger'], sample['flags']
        if type(trigger) not in (int, float) or not math.isfinite(trigger) or not 0 <= trigger <= 1:
            raise ValueError(f'Sample {index} trigger invalid')
        if type(flags) is not int or flags < 0 or flags & ~1023:
            raise ValueError(f'Sample {index} flags invalid')
        phase, expected_trigger, expected_flags = phase_input(index)
        if (sample['phase'], trigger, flags) != (phase, expected_trigger, expected_flags):
            raise ValueError(f'Sample {index} does not follow the full control/loss/rearm script')


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return [w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u,
            w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s]


def compose(a, b):
    position, rotation = a
    child_position, child_rotation = b
    rotated = multiply(multiply(rotation, [*child_position, 0.]),
                       [-rotation[0], -rotation[1], -rotation[2], rotation[3]])
    return ([position[i] + rotated[i] for i in range(3)], multiply(rotation, child_rotation))


def axis_rotation(axis, angle):
    length = math.sqrt(sum(v*v for v in axis))
    if length < 1e-12:
        raise ValueError('Invalid URDF axis')
    return [*(v / length * math.sin(angle / 2) for v in axis), math.cos(angle / 2)]


def urdf_chain(source):
    """Read the actual source hierarchy, including joint8 and fixed palm joint."""
    root = ET.parse(source).getroot()
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain, link = [], 'panda_hand'
    while link != 'panda_link0':
        joint = by_child[link]
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        roll, pitch, yaw = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        rotation = multiply(multiply(axis_rotation([0, 0, 1], yaw),
                                     axis_rotation([0, 1, 0], pitch)), axis_rotation([1, 0, 0], roll))
        axis = joint.find('axis')
        chain.append((joint.get('name'), joint.get('type'), (xyz, rotation),
                      [float(v) for v in axis.get('xyz').split()] if axis is not None else None))
        link = joint.find('parent').get('link')
    return list(reversed(chain))


def source_fk(chain, joints):
    pose = ([0., 0., 0.], [0., 0., 0., 1.])
    for name, kind, origin, axis in chain:
        pose = compose(pose, origin)
        if kind == 'revolute':
            pose = compose(pose, ([0., 0., 0.], axis_rotation(axis, joints[name])))
        elif kind != 'fixed':
            raise ValueError(f'Unexpected palm chain joint type: {kind}')
    return pose


def create_source(description_root, trace_path, manifest):
    source = Path(description_root) / 'robots/panda_arm_hand.urdf'
    if hashlib.sha256(source.read_bytes()).hexdigest() != manifest['urdf_sha256']:
        raise ValueError('Source URDF hash differs from physics manifest')
    chain = urdf_chain(source)
    home = manifest['initial_q'][:7]
    samples = []
    for index in range(SAMPLE_COUNT):
        # Modest smooth independent variations, starting exactly at home. The
        # palm remains reachable because every grip pose comes from source FK.
        time = max(0, index - 29) * .01
        amplitudes = [.08, .06, .07, .07, .08, .07, .07]
        q = [home[j] + amplitudes[j] * math.sin(time * (.55 + .09*j)) for j in range(7)]
        palm = source_fk(chain, dict(zip(manifest['joint_names'][:7], q)))
        position, rotation = compose(([0., 0., -1.2], [-.5, .5, .5, .5]), palm)
        norm = math.sqrt(sum(v*v for v in rotation))
        phase, trigger, flags = phase_input(index)
        samples.append(dict(index=index, position=position, rotation=[v/norm for v in rotation],
                            trigger=trigger, flags=flags, phase=phase))
    data = dict(schema_version=1, sample_count=SAMPLE_COUNT, timestep_seconds=.001,
                substeps_per_sample=SUBSTEPS, urdf_sha256=manifest['urdf_sha256'], samples=samples)
    validate_trace(data, manifest)
    trace_path = Path(trace_path)
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    trace_path.write_text(canonical_json(data), encoding='utf-8', newline='\n')


def cpp_float(value):
    value = format(value, '.9g')
    if '.' not in value and 'e' not in value:
        value += '.0'
    return value + 'F'


def generate(trace_path, manifest_path, output):
    trace_bytes, manifest_bytes = Path(trace_path).read_bytes(), Path(manifest_path).read_bytes()
    data, manifest = json.loads(trace_bytes), json.loads(manifest_bytes)
    validate_trace(data, manifest)
    header = ['#pragma once', '#include "controller_trace.h"', '#include <array>', '#include <cstddef>',
              'namespace quest_newton::generated_trace {',
              f'inline constexpr std::size_t kSubstepsPerSample = {SUBSTEPS};',
              f'inline constexpr char kTraceSha256[] = "{hashlib.sha256(trace_bytes).hexdigest()}";',
              f'inline constexpr char kPhysicsManifestSha256[] = "{hashlib.sha256(manifest_bytes).hexdigest()}";',
              f'inline constexpr std::array<verification::RecordedControllerSample, {SAMPLE_COUNT}> kSamples{{{{']
    for sample in data['samples']:
        position = ', '.join(map(cpp_float, sample['position']))
        rotation = ', '.join(map(cpp_float, sample['rotation']))
        header.append('    {{{' + position + '}, {' + rotation + '}}, ' +
                      cpp_float(sample['trigger']) + ', ' + str(sample['flags']) + 'U},')
    header.extend(['}};', '} // namespace quest_newton::generated_trace', ''])
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'controller_trace_data.h').write_text('\n'.join(header), encoding='utf-8', newline='\n')
    (output / 'controller_trace.json').write_bytes(trace_bytes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--artifact-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--create-source', action='store_true')
    parser.add_argument('--franka-description-root', type=Path)
    args = parser.parse_args()
    if args.create_source:
        if not args.franka_description_root:
            parser.error('--create-source requires --franka-description-root')
        create_source(args.franka_description_root, args.trace, json.loads(args.artifact_manifest.read_bytes()))
    generate(args.trace, args.artifact_manifest, args.output)


if __name__ == '__main__':
    main()
