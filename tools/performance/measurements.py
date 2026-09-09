"""Interpret actual benchmark counters; missing evidence is never a zero."""
import math
import re


def number(value):
    return value if type(value) in (int, float) and math.isfinite(value) else None


def evaluate(native, perfetto=None):
    wall = number(native.get('wall_seconds'))
    simulation = number(native.get('simulation_seconds'))
    steps = number(native.get('successful_steps'))
    rate = number(native.get('requested_physics_hz'))
    if wall is None or wall <= 0 or simulation is None or steps is None or rate is None or rate <= 0:
        return dict(valid=False, requested_hz=rate, achieved_hz=None,
                    realtime_ratio=None, lag_seconds=None, discarded_fraction=None,
                    native_render_hz=None, native_xr_coverage=False,
                    physics_p99_us=None, physics_budget_us=1e6/rate if rate and rate > 0 else None,
                    p99_headroom=False, physics_sustainable=False, recommended_candidate=False,
                    worker_busy_fraction=None, external_xr=perfetto,
                    reason=native.get('failure') or 'incomplete counters')
    ratio = simulation / wall
    discarded = number(native.get('dropped_wall_seconds'))
    frames = number(native.get('frame_count'))
    physics = native.get('physics_timing', {})
    p99 = number(physics.get('p99_us'))
    dt_us = 1e6 / rate
    finite = native.get('phase') == 'complete' and native.get('finite') is True and native.get('passed') is True
    count_ok = physics.get('count') == steps and abs(simulation - steps / rate) <= max(1e-5, simulation * 1e-7)
    sustainable = finite and count_ok and ratio >= .99 and discarded is not None and discarded / wall <= .001
    headroom = p99 is not None and p99 <= dt_us
    fps = frames / wall if frames is not None else None
    native_xr = fps is not None and fps >= 90 * .98 and abs((number(native.get('refresh_hz')) or 0) - 90) <= .1
    busy = 0.
    for name in ('physics_timing', 'ik_timing', 'base_timing'):
        summary = native.get(name, {})
        mean, count = number(summary.get('mean_us')), number(summary.get('count'))
        if mean is not None and count is not None:
            busy += mean * count / 1e6
    return dict(valid=finite and count_ok, requested_hz=rate, achieved_hz=steps / wall,
                realtime_ratio=ratio, lag_seconds=wall-simulation,
                discarded_fraction=discarded / wall if discarded is not None else None,
                native_render_hz=fps, native_xr_coverage=native_xr,
                physics_p99_us=p99, physics_budget_us=dt_us, p99_headroom=headroom,
                physics_sustainable=sustainable, recommended_candidate=sustainable and headroom and native_xr,
                worker_busy_fraction=busy / wall,
                external_xr=perfetto,
                reason='complete measured pipeline' if finite and count_ok else native.get('failure', 'invalid pipeline'))


def health(battery_text, thermal_text, memory_text):
    def value(text, pattern, divisor=1):
        match = re.search(pattern, text, re.M)
        return float(match.group(1)) / divisor if match else None
    current = thermal_text.split('Current temperatures from HAL:', 1)[-1].split('Current cooling devices', 1)[0]
    temperatures = {}
    for match in re.finditer(r'Temperature\{mValue=([-\d.]+), mType=(\d+), mName=([^,}]+), mStatus=(\d+)\}', current):
        temperatures[match[3]] = dict(celsius=float(match[1]), type=int(match[2]), status=int(match[4]))
    def maximum(kind):
        values = [row['celsius'] for row in temperatures.values() if row['type'] == kind]
        return max(values) if values else None
    powered = re.findall(r'(?:AC|USB|Wireless|Dock) powered:\s*(true|false)', battery_text)
    return dict(battery_percent=value(battery_text, r'^\s*level:\s*(\d+)'),
                battery_celsius=value(battery_text, r'^\s*temperature:\s*(\d+)', 10),
                charging=('true' in powered) if powered else None,
                thermal_status=value(thermal_text, r'Thermal Status:\s*(\d+)'),
                cpu_max_celsius=maximum(0), gpu_max_celsius=maximum(1),
                surface_celsius=temperatures.get('surf-virt-usr', {}).get('celsius'),
                pss_kb=value(memory_text, r'TOTAL PSS:\s*(\d+)'),
                rss_kb=value(memory_text, r'TOTAL RSS:\s*(\d+)'), temperatures=temperatures)
