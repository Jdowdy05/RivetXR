"""Validate nonce-bound native replay evidence; never infer external performance gates."""
import argparse
import json
import math
from pathlib import Path
import re
import sys

from tools.verification.generate_controller_trace import contract_manifest
from tools.verification.state_limits import (
    ALLOWANCES, StateLimitStats, float32, validate_policy, validate_statistics,
)

LOWER = tuple(map(float32, contract_manifest()['lower_limits']))
UPPER = tuple(map(float32, contract_manifest()['upper_limits']))
Q_TOLERANCE = 1e-5
QD_TOLERANCE = 1e-4
REMAINING_EVIDENCE = ['60-second Perfetto trace', '30-minute soak with thermal and frequency analysis',
                      '90 Hz frame-budget attribution, missed frames, PSS and battery measurements',
                      'live Touch tracking and visual acceptance']


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value, label):
    require(type(value) in (int, float) and math.isfinite(value), f'{label} must be finite numeric')
    return value


def integer(value, label, minimum=0):
    require(type(value) is int and value >= minimum, f'{label} must be an integer >= {minimum}')
    return value


def vector(value, length, label):
    require(isinstance(value, list) and len(value) == length, f'{label} needs {length} values')
    for entry in value:
        number(entry, label)
    return value


def identity(document, expected, mode, run_id):
    require(isinstance(document, dict), 'document must be an object')
    validate_policy(document.get('state_limit_policy'))
    validate_policy(expected.get('state_limit_policy'))
    require(isinstance(run_id, str) and re.fullmatch(r'[a-zA-Z0-9_-]{1,64}', run_id), 'invalid run ID')
    require(type(document.get('schema_version')) is int and document['schema_version'] == 1, 'schema_version mismatch')
    require(document.get('mode') == mode, 'mode mismatch')
    require(document.get('run_id') == run_id, 'run_id mismatch; stale evidence')
    for key in ('trace_sha256', 'physics_manifest_sha256'):
        digest = expected.get(key)
        require(isinstance(digest, str) and re.fullmatch('[a-f0-9]{64}', digest), f'invalid expected {key}')
        require(document.get(key) == digest, f'{key} mismatch')


def timing(summary, label, minimum_count=1, exact_count=None, required=True):
    require(isinstance(summary, dict), f'{label} timing missing')
    require(type(summary.get('available')) is bool, f'{label} available must be boolean')
    count = integer(summary.get('count'), f'{label} count')
    values = {key: number(summary.get(key), f'{label} {key}') for key in ('mean_us', 'p95_us', 'p99_us', 'max_us')}
    over = integer(summary.get('over_budget_count'), f'{label} over_budget_count')
    require(over <= count, f'{label} over_budget_count exceeds count')
    require(0 <= values['mean_us'] <= values['max_us'] and
            0 <= values['p95_us'] <= values['p99_us'] <= values['max_us'], f'{label} timing ordering invalid')
    if not summary['available']:
        require(not required and count == 0 and over == 0 and all(v == 0 for v in values.values()), f'{label} timing unavailable')
        return
    require(count >= minimum_count and values['max_us'] > 0, f'{label} timing coverage missing')
    if exact_count is not None:
        require(count == exact_count, f'{label} count must be {exact_count}')
    if label == 'physics':
        require(values['p99_us'] < 2000, 'physics p99 must be below 2000 us')


def states(rows, label, native=False):
    require(isinstance(rows, list) and len(rows) == 1000, f'{label} needs exactly 1000 states')
    previous = None
    observed = StateLimitStats()
    for index, row in enumerate(rows):
        require(isinstance(row, dict), f'{label} state must be an object')
        require(type(row.get('index')) is int and row['index'] == index, f'{label} index order mismatch at {index}')
        should_engage = any(a <= index <= b for a, b in ((30, 349), (380, 399), (440, 599), (640, 849), (900, 979)))
        require(type(row.get('engaged')) is bool and row['engaged'] == should_engage, f'{label} engaged phase incorrect at {index}')
        require(type(row.get('calibrated')) is bool and row['calibrated'] == (index >= 1), f'{label} calibrated phase incorrect at {index}')
        for key, size in (('targets', 7), ('q', 9), ('qd', 9)):
            vector(row.get(key), size, f'{label} {key} at {index}')
        require(all(low <= value <= high for value, low, high in zip(row['targets'], LOWER, UPPER)),
                f'{label} targets joint limit at {index}')
        excursions = observed.observe(row['q'], LOWER, UPPER)
        require(all(excess <= allowance for excess, allowance in zip(excursions, ALLOWANCES)),
                f'{label} q state allowance at {index}')
        if previous is not None and not row['engaged']:
            require(row['targets'] == previous['targets'], f'{label} inactive targets changed at {index}')
        if native:
            require(row.get('type') == 'state', f'{label} state type missing')
            require(abs(number(row.get('sim_time_s'), 'sim_time_s') - .01 * (index + 1)) <= 1e-6,
                    f'{label} simulation time mismatch at {index}')
            body = vector(row.get('body_q'), 84, f'{label} body_q at {index}')
            for offset in range(0, 84, 7):
                require(abs(sum(v * v for v in body[offset + 3:offset + 7]) - 1.) <= 1e-4,
                        f'{label} body quaternion not normalized at {index}')
        previous = row
    for field in ('q', 'targets'):
        require(max(max(row[field][j] for row in rows) - min(row[field][j] for row in rows) for j in range(7)) > .01,
                f'{label} {field} has no meaningful motion')
    return observed


def expected_contract(expected):
    require(isinstance(expected, dict) and type(expected.get('schema_version')) is int and expected['schema_version'] == 1,
            'expected schema_version mismatch')
    require(expected.get('tolerances') == {'joint_q': Q_TOLERANCE, 'joint_qd': QD_TOLERANCE},
            'expected tolerances must be joint_q=1e-5 and joint_qd=1e-4')
    validate_policy(expected.get('state_limit_policy'))
    require(type(expected.get('physics_substeps')) is int and expected['physics_substeps'] == 10000 and
            type(expected.get('substeps_per_sample')) is int and expected['substeps_per_sample'] == 10 and
            expected.get('timestep_seconds') == .001, 'expected physics step contract mismatch')
    observed = states(expected.get('samples'), 'expected')
    validate_statistics(expected.get('state_limits'), 10000, observed)


def native_trace(expected, rows, run_id, label):
    require(isinstance(rows, list) and len(rows) == 1002, f'{label} requires header, 1000 states and footer')
    header, footer = rows[0], rows[-1]
    identity(header, expected, 'trace', run_id)
    require(header.get('type') == 'header' and type(header.get('state_count')) is int and header['state_count'] == 1000
            and type(header.get('substeps_per_sample')) is int and header['substeps_per_sample'] == 10, f'{label} header count/type mismatch')
    require(isinstance(footer, dict) and footer.get('type') == 'complete' and
            type(footer.get('state_count')) is int and footer['state_count'] == 1000, f'{label} completion missing')
    timing(footer.get('physics'), 'physics', exact_count=10000)
    samples = rows[1:-1]
    observed = states(samples, label, native=True)
    validate_statistics(footer.get('state_limits'), footer['physics']['count'], observed)
    errors = {'joint_q': 0., 'joint_qd': 0., 'targets': 0.}
    for actual, reference in zip(samples, expected['samples']):
        for field, key, tolerance in (('q', 'joint_q', Q_TOLERANCE), ('qd', 'joint_qd', QD_TOLERANCE), ('targets', 'targets', Q_TOLERANCE)):
            error = max(abs(a - b) for a, b in zip(actual[field], reference[field]))
            errors[key] = max(errors[key], error)
            require(error <= tolerance, f'{label} {field} differs from expected at {actual["index"]}: {error} > {tolerance}')
    return samples, errors


def verify_runs(expected, trace_a, trace_b, run_id_a, run_id_b):
    expected_contract(expected)
    require(run_id_a != run_id_b, 'Android runs must have distinct fresh run IDs')
    a, errors_a = native_trace(expected, trace_a, run_id_a, 'trace A')
    b, errors_b = native_trace(expected, trace_b, run_id_b, 'trace B')
    difference = max(abs(x - y) for left, right in zip(a, b) for x, y in zip(left['q'], right['q']))
    require(difference <= Q_TOLERANCE, f'Android repeatability error {difference} exceeds 1e-5')
    return dict(functional_pass=True, performance_complete=False, state_count=1000,
                run_ids=[run_id_a, run_id_b], trace_sha256=expected['trace_sha256'],
                physics_manifest_sha256=expected['physics_manifest_sha256'],
                maximum_expected_errors={'a': errors_a, 'b': errors_b}, maximum_android_q_error=difference,
                physics={'a': trace_a[-1]['physics'], 'b': trace_b[-1]['physics']},
                state_limit_policy=expected['state_limit_policy'],
                state_limits={'expected': expected['state_limits'], 'a': trace_a[-1]['state_limits'],
                              'b': trace_b[-1]['state_limits']},
                remaining_evidence=REMAINING_EVIDENCE)


def verify_metrics(expected, metrics, run_id, minimum_soak_seconds=1800):
    expected_contract(expected)
    identity(metrics, expected, 'soak', run_id)
    require(metrics.get('completed') is True, 'soak is not completed')
    requested = integer(metrics.get('soak_seconds_requested'), 'soak_seconds_requested', 1)
    require(number(minimum_soak_seconds, 'minimum_soak_seconds') >= 1, 'minimum soak must be positive')
    duration = number(metrics.get('duration_seconds'), 'duration_seconds')
    require(duration >= max(requested, minimum_soak_seconds), 'actual soak duration is too short')
    count = integer(metrics.get('valid_state_count'), 'valid_state_count', 1000)
    refresh = number(metrics.get('refresh_hz'), 'refresh_hz')
    require(abs(refresh - 90.) <= .1, 'soak refresh must be 90 Hz')
    frames = integer(metrics.get('render_frames'), 'render_frames', 1)
    # Coverage is a liveness gate. External frame-time attribution remains required.
    require(frames >= duration * refresh * .5, 'soak render coverage below 50 percent of display cadence')
    timing(metrics.get('physics'), 'physics', exact_count=count * 10)
    validate_statistics(metrics.get('state_limits'), metrics['physics']['count'])
    timing(metrics.get('render_cpu'), 'render_cpu', exact_count=frames)
    require(type(metrics.get('gpu_timer_supported')) is bool, 'gpu_timer_supported must be boolean')
    integer(metrics.get('gpu_disjoint_count'), 'gpu_disjoint_count')
    timing(metrics.get('render_gpu'), 'render_gpu', required=metrics['gpu_timer_supported'])
    require(metrics['render_gpu']['count'] <= frames, 'render_gpu count exceeds render_frames')
    require(metrics['render_gpu']['available'] == metrics['gpu_timer_supported'], 'GPU timer availability mismatch')
    return dict(native_soak_evidence=True, performance_complete=False, run_id=run_id,
                duration_seconds=duration, metrics=metrics, remaining_evidence=REMAINING_EVIDENCE)


def load_json(path, lines=False):
    def invalid(value):
        raise ValueError(f'non-finite JSON constant {value}')
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, f'duplicate JSON key {key}')
            result[key] = value
        return result
    text = Path(path).read_text(encoding='utf-8-sig')
    parse = lambda value: json.loads(value, parse_constant=invalid, object_pairs_hook=unique)
    return [parse(line) for line in text.splitlines()] if lines else parse(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected', required=True)
    for name in ('trace-a', 'trace-b', 'run-id-a', 'run-id-b', 'metrics', 'run-id'):
        parser.add_argument('--' + name)
    parser.add_argument('--minimum-soak-seconds', type=int, default=1800)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        expected = load_json(args.expected)
        if args.metrics:
            require(args.run_id and not any((args.trace_a, args.trace_b, args.run_id_a, args.run_id_b)), 'metrics requires --run-id and no trace arguments')
            report = verify_metrics(expected, load_json(args.metrics), args.run_id, args.minimum_soak_seconds)
        else:
            require(all((args.trace_a, args.trace_b, args.run_id_a, args.run_id_b)) and not args.run_id,
                    'trace comparison requires both traces and both run IDs')
            report = verify_runs(expected, load_json(args.trace_a, True), load_json(args.trace_b, True), args.run_id_a, args.run_id_b)
    except (ValueError, OSError, TypeError, KeyError) as error:
        report = dict(functional_pass=False, native_soak_evidence=False, performance_complete=False, error=str(error))
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    Path(args.output).write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('Native soak evidence passed; external performance analysis remains required.' if args.metrics else
          'Functional recorded replay passed; external performance and live Touch acceptance remain required.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
