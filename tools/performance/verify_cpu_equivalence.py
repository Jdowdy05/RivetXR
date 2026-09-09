"""Exact host CPU replay against a git-pinned runtime; never drives a device.

Run with the pinned Newton/Warp host environment and real Franka asset root.
Correctness replay and unprofiled host timing are separate passes. A failed
baseline/baseline control blocks candidate claims. No numerical tolerance is used.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import logging
import math
from pathlib import Path
import struct
import subprocess
import sys
import time
import traceback
import types


REPO = Path(__file__).resolve().parents[2]
RUNTIME_PATH = 'quest/app/src/main/python/quest_sim/runtime.py'
# The exact comparison implementation is packaged for Android as well.
_PACKAGE_ROOT = str(REPO / 'quest/app/src/main/python')
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)
from quest_sim import cpu_validation
from quest_sim.cpu_validation import (
    apply_operation, capture_session, compare_replay, first_difference,
    json_safe, operation_schedule, snapshot_physics_bytes,
)


def timing_pass(factory, assets, steps):
    """Unprofiled host wall timings; no state capture/hash/comparison in this loop."""
    session = factory(str(assets), json.dumps(dict(initial_base_pose=[0., 0., .7, 0., 0., 0., 1.])))
    session.command('spawn_box')
    operations = []
    step_count = 0
    for operation in operation_schedule(steps):
        if operation['kind'] in ('base', 'step'):
            operations.append(operation)
            step_count += operation['kind'] == 'step'
        if step_count == steps:
            break
    for operation in operations[:min(100, len(operations))]:
        apply_operation(session, operation)
    session.set_base_pose([0., 0., .7, 0., 0., 0., 1.]); session.reset()
    totals, counts = collections.defaultdict(float), collections.Counter()
    start = time.perf_counter()
    for operation in operations:
        before = time.perf_counter()
        apply_operation(session, operation)
        totals[operation['kind']] += time.perf_counter() - before
        counts[operation['kind']] += 1
    wall = time.perf_counter() - start
    return dict(wall_seconds=wall, operation_counts=dict(counts), timed_seconds=dict(totals),
                mean_us={kind: totals[kind] * 1e6 / counts[kind] for kind in counts},
                metadata=json.loads(session.metadata()),
                scope='Unpaced host wall timings, no profiler or equivalence capture; not Quest performance')


def _load_module(source, name, filename):
    # Relative imports in the candidate resolve through the repository package.
    package_root = str(REPO / 'quest/app/src/main/python')
    if package_root not in sys.path:
        sys.path.insert(0, package_root)
    module = types.ModuleType('quest_sim.' + name)
    module.__file__, module.__package__ = filename, 'quest_sim'
    sys.modules[module.__name__] = module
    exec(compile(source, filename, 'exec'), module.__dict__)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--franka-description-root', required=True, type=Path)
    parser.add_argument('--baseline-ref', default='57da647')
    parser.add_argument('--candidate-runtime', type=Path, default=REPO / RUNTIME_PATH)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--steps', type=int, default=640, help='Main motion steps; other lifecycle segments are added')
    parser.add_argument('--timing-steps', type=int, default=1200)
    parser.add_argument('--skip-timing', action='store_true')
    args = parser.parse_args()
    if args.steps < 8 or args.timing_steps < 8:
        parser.error('At least eight steps are required')
    report = dict(schema_version=1, scope='Host CPU exact reference/candidate comparison; no Quest claims',
                  passed=False, baseline_ref=args.baseline_ref,
                  comparison='Exact dtype, shape and bytes; only snapshot CPU time bytes40:48 excluded',
                  contact_policy='Compare named fields of active contacts; no C-struct padding or inactive capacity',
                  python=sys.version, argv=sys.argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        asset_root = args.franka_description_root.resolve()
        if (asset_root / 'franka_description').is_dir():
            asset_root = asset_root / 'franka_description'
        asset_urdf = asset_root / 'robots/panda_arm_hand.urdf'
        if not asset_urdf.is_file():
            raise FileNotFoundError('Real Franka URDF required: ' + str(asset_urdf))
        report['franka_urdf'] = str(asset_urdf)
        report['franka_urdf_sha256'] = hashlib.sha256(asset_urdf.read_bytes()).hexdigest()
        report['checker_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        source = subprocess.check_output(['git', 'show', f'{args.baseline_ref}:{RUNTIME_PATH}'], cwd=REPO)
        candidate_source = args.candidate_runtime.read_bytes()
        report['baseline_source_sha256'] = hashlib.sha256(source).hexdigest()
        report['baseline_commit'] = subprocess.check_output(['git', 'rev-parse', args.baseline_ref], cwd=REPO, text=True).strip()
        report['candidate_source_sha256'] = hashlib.sha256(candidate_source).hexdigest()
        report['candidate_runtime'] = str(args.candidate_runtime.resolve())
        report['candidate_support_sources'] = {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                                               for path in (REPO / 'quest/app/src/main/python/quest_sim').glob('*.py')}
        baseline = _load_module(source, '_equivalence_reference', f'<git:{args.baseline_ref}:{RUNTIME_PATH}>')
        candidate = _load_module(candidate_source, '_equivalence_candidate', str(args.candidate_runtime))
        logging.getLogger('trimesh').setLevel(logging.ERROR)
        operations = operation_schedule(args.steps)
        report['operation_counts'] = dict(collections.Counter(op['kind'] for op in operations))
        report['operations_sha256'] = hashlib.sha256(json.dumps(operations, sort_keys=True).encode()).hexdigest()
        report['reference_control'] = compare_replay(baseline.create, baseline.create, args.franka_description_root, operations)
        if report['reference_control']['passed']:
            report['candidate_comparison'] = compare_replay(baseline.create, candidate.create, args.franka_description_root, operations)
            report['passed'] = report['candidate_comparison']['passed']
        if report['passed'] and not args.skip_timing:
            # ABBA ordering exposes drift without pooling correctness and timing.
            report['host_timings'] = []
            for label, factory in [('reference', baseline.create), ('candidate', candidate.create),
                                   ('candidate', candidate.create), ('reference', baseline.create)]:
                report['host_timings'].append(dict(implementation=label,
                                                   **timing_pass(factory, args.franka_description_root, args.timing_steps)))
        current_support = {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                           for path in (REPO / 'quest/app/src/main/python/quest_sim').glob('*.py')}
        report['candidate_sources_unchanged_during_run'] = (
            current_support == report['candidate_support_sources'] and
            hashlib.sha256(args.candidate_runtime.read_bytes()).hexdigest() == report['candidate_source_sha256'])
        if not report['candidate_sources_unchanged_during_run']:
            raise RuntimeError('Candidate source changed during verification; rerun against stable files')
    except Exception as exception:
        report['passed'] = False
        report['error'] = dict(type=type(exception).__name__, message=str(exception), traceback=traceback.format_exc())
    args.output.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], output=str(args.output.resolve()),
                          reference_control=report.get('reference_control', {}).get('passed'))))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
