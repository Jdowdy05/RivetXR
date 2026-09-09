"""Approved zero-force replay state allowances; targets retain strict bounds.

Coordinates are never changed. Excursions are relative to float32 manifest
bounds, in radians for the seven arm joints and metres for both fingers.
"""
import math
import struct

STATE_LIMIT_POLICY = dict(name='newton_soft_stop_state_v1', arm_rad=1e-5,
                          finger_m=1e-6, target_limits='strict')
ALLOWANCES = (1e-5,) * 7 + (1e-6,) * 2
UINT64_MAX = (1 << 64) - 1


def float32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def validate_policy(policy):
    if (not isinstance(policy, dict) or policy != STATE_LIMIT_POLICY or
            any(type(policy[key]) is not type(value) for key, value in STATE_LIMIT_POLICY.items())):
        raise ValueError('state_limit_policy must match the approved newton_soft_stop_state_v1 policy')


def _vector(values, label):
    if (not isinstance(values, (list, tuple)) or len(values) != 9 or
            any(type(value) not in (int, float) or not math.isfinite(value) for value in values)):
        raise ValueError(f'{label} must contain nine finite numbers')


def measure_excursions(q, lower, upper):
    for values, label in ((q, 'q'), (lower, 'lower limits'), (upper, 'upper limits')):
        _vector(values, label)
    try:
        lower, upper = list(map(float32, lower)), list(map(float32, upper))
    except (OverflowError, struct.error) as error:
        raise ValueError('state bounds must be finite float32 numbers') from error
    if any(not math.isfinite(lo) or not math.isfinite(hi) or lo > hi for lo, hi in zip(lower, upper)):
        raise ValueError('state bounds must be finite and ordered')
    return [max(0., lo - value, value - hi) for value, lo, hi in zip(q, lower, upper)]


class StateLimitStats:
    """Accumulate every physics substep, including accepted raw excursions."""
    def __init__(self):
        self.samples_checked = 0
        self.strict_excursion_samples = 0
        self.per_joint_excursion_counts = [0] * 9
        self.max_excursions = [0.] * 9
        self.allowance_violations = 0

    def observe(self, q, lower, upper):
        excursions = measure_excursions(q, lower, upper)
        self.samples_checked += 1
        self.strict_excursion_samples += any(value > 0 for value in excursions)
        self.allowance_violations += any(value > allowance for value, allowance in zip(excursions, ALLOWANCES))
        for joint, value in enumerate(excursions):
            self.per_joint_excursion_counts[joint] += value > 0
            self.max_excursions[joint] = max(self.max_excursions[joint], value)
        return excursions

    def as_dict(self):
        return dict(samples_checked=self.samples_checked, strict_excursion_samples=self.strict_excursion_samples,
                    per_joint_excursion_counts=self.per_joint_excursion_counts[:],
                    max_excursions=self.max_excursions[:], allowance_violations=self.allowance_violations)


def _count(value, label):
    if type(value) is not int or not 0 <= value <= UINT64_MAX:
        raise ValueError(f'state_limits {label} must be a uint64 count')
    return value


def validate_statistics(stats, expected_count, observed=None):
    """Validate full-substep evidence against observed and omitted substeps."""
    if not isinstance(stats, dict) or set(stats) != set(StateLimitStats().as_dict()):
        raise ValueError('state_limits statistics fields missing or different')
    count = _count(stats['samples_checked'], 'samples_checked')
    if count != _count(expected_count, 'expected count'):
        raise ValueError('state_limits samples_checked count differs from physics count')
    raw = _count(stats['strict_excursion_samples'], 'strict_excursion_samples')
    violations = _count(stats['allowance_violations'], 'allowance_violations')
    counts, maxima = stats['per_joint_excursion_counts'], stats['max_excursions']
    if not isinstance(counts, list) or len(counts) != 9:
        raise ValueError('state_limits needs nine per-joint excursion counts')
    _vector(maxima, 'state_limits max_excursions')
    for value in counts:
        _count(value, 'per_joint_excursion_counts')
    if (not max(counts) <= raw <= min(count, sum(counts)) or violations != 0 or
            any(not 0 <= maximum <= allowance or (maximum == 0) != (joint_count == 0)
                for maximum, joint_count, allowance in zip(maxima, counts, ALLOWANCES))):
        raise ValueError('state_limits excursion counts, maxima or allowance violations inconsistent')
    if observed is not None:
        observed = observed.as_dict() if isinstance(observed, StateLimitStats) else observed
        if (count < observed['samples_checked'] or raw < observed['strict_excursion_samples'] or
                any(a < b for a, b in zip(counts, observed['per_joint_excursion_counts'])) or
                any(a < b for a, b in zip(maxima, observed['max_excursions']))):
            raise ValueError('state_limits underreports observed raw excursions')
        omitted = count - observed['samples_checked']
        residual_any = raw - observed['strict_excursion_samples']
        residual_joints = [a - b for a, b in zip(counts, observed['per_joint_excursion_counts'])]
        if (not 0 <= residual_any <= omitted or
                any(not 0 <= value <= omitted for value in residual_joints) or
                not max(residual_joints) <= residual_any <= min(omitted, sum(residual_joints))):
            raise ValueError('state_limits omitted-substep excursion counts have an impossible union')
        if any(residual == 0 and maximum != observed_maximum
               for residual, maximum, observed_maximum in
               zip(residual_joints, maxima, observed['max_excursions'])):
            raise ValueError('state_limits maximum changed without an omitted excursion')
    return stats
