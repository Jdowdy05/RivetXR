"""Export measured trials to CSV/JSON and standalone publication files."""
import argparse
import csv
import json
from pathlib import Path

from tools.performance.measurements import evaluate


def collect(root):
    rows, evidence = [], []
    paths = sorted(root.glob('*/*/trial.json'), key=lambda p: json.loads(p.read_text())['started_utc'])
    for path in paths:
        trial = json.loads(path.read_text(encoding='utf-8'))
        native = trial['native']
        assessment = evaluate(native)
        trace_path = path.with_name('perfetto.json')
        trace = json.loads(trace_path.read_text()) if trace_path.exists() else None
        row = dict(run_id=native['run_id'], group=path.parent.parent.name,
                   workload=native['workload'], traced=trial['traced'],
                   configured_control_hz=native['configured_control_hz'],
                   configured_publication_hz=native['configured_publication_hz'],
                   achieved_control_hz=native['achieved_control_hz'],
                   achieved_publication_hz=native['achieved_publication_hz'],
                   wall_seconds=native['wall_seconds'], simulation_seconds=native['simulation_seconds'],
                   successful_steps=native['successful_steps'], dropped_wall_seconds=native['dropped_wall_seconds'],
                   box_settled_floor_observed=native['box_settled_floor_observed'],
                   max_robot_body_displacement_m=native['max_robot_body_displacement_m'],
                   global_contact_max=native['global_contact_max'],
                   gpu_supported=native['gpu_supported'], gpu_disjoint_count=native['gpu_disjoint_count'],
                   **{key: value for key, value in assessment.items() if key not in ('external_xr', 'reason')})
        for name in ('physics', 'base', 'ik', 'xr_cpu', 'gpu'):
            timing = native[name + '_timing']
            for statistic in ('count', 'mean_us', 'p95_us', 'p99_us', 'max_us', 'over_budget_count'):
                row[name + '_' + statistic] = timing[statistic]
        for side in ('before', 'after'):
            health = trial.get('health_' + side) or {}
            for name in ('cpu_max_celsius', 'gpu_max_celsius', 'surface_celsius', 'battery_celsius',
                         'thermal_status', 'pss_kb', 'rss_kb', 'battery_percent', 'charging'):
                row[side + '_' + name] = health.get(name)
        row['trace_quality_eligible'] = trace.get('strong_xr_evidence_eligible') if trace else None
        row['trace_stale_mean_fraction'] = trace.get('stale_evidence', {}).get('appinfo_mean_fraction_of_90hz') if trace else None
        row['trace_path'] = str(trace_path.relative_to(root)) if trace else None
        row['evidence_path'] = str(path.relative_to(root))
        rows.append(row)
        evidence.append(dict(trial=trial, evaluation=assessment, perfetto=trace))
    if not rows:
        raise ValueError('No completed trial.json files found at <root>/<group>/<trial>/')
    return rows, evidence


def plot(rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import PercentFormatter
    import numpy as np

    failed_count = sum(not r['valid'] for r in rows)
    rows = [r for r in rows if r['valid']]
    if not rows:
        raise ValueError('No valid measured windows to plot; failed evidence remains in CSV/JSON')

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.titleweight': 'bold', 'axes.titlepad': 12,
                         'axes.labelcolor': '#334155', 'text.color': '#172033',
                         'axes.edgecolor': '#CBD5E1', 'grid.color': '#E2E8F0'})
    colors = {'pass': '#17806D', 'fail': '#D45C3D', 'rest': '#6955A2'}
    fig, axs = plt.subplots(2, 2, figsize=(13.5, 9))
    fig.subplots_adjust(top=.84, bottom=.16, hspace=.5, wspace=.26)
    fig.suptitle('Quest 3S · Newton CPU physics rate sweep', x=.07, ha='left', y=.975,
                 fontsize=23, fontweight='bold')
    fig.text(.07, .923, 'One Franka + one dynamic box, native OpenXR at 90 Hz · measured on the headset', fontsize=12)
    fig.text(.07, .885, 'Motion includes arm IK, analog gripper and base-height updates. Integer control/publication cadence stays near 100 Hz.',
             fontsize=10, color='#475569')
    for ax in axs.flat:
        ax.grid(axis='y', zorder=0)
    motion = [r for r in rows if r['workload'] == 'motion']
    largest = max(r['requested_hz'] for r in rows)
    ax = axs[0, 0]
    ax.plot([0, largest], [0, largest], color='#94A3B8', linestyle='--', lw=1, label='Real time')
    for row in rows:
        color = colors['rest' if row['workload'] == 'rest' else ('pass' if row['recommended_candidate'] else 'fail')]
        marker = 's' if row['workload'] == 'rest' else 'o'
        ax.scatter(row['requested_hz'], row['achieved_hz'], s=40, marker=marker,
                   facecolors='none' if row['traced'] else color, edgecolors=color, lw=1.4, zorder=3)
    ax.set(xlabel='Requested physics rate (steps/s)', ylabel='Achieved physics rate (steps/s)',
           title='How much physics actually runs', xlim=(0, largest * 1.04), ylim=(0, largest * 1.04))

    ax = axs[0, 1]
    for row in motion:
        color = colors['pass' if row['recommended_candidate'] else 'fail']
        ax.scatter(row['requested_hz'], row['realtime_ratio'] * 100, s=40,
                   facecolors='none' if row['traced'] else color, edgecolors=color, lw=1.4, zorder=3)
    ax.axhline(99, color='#94A3B8', linestyle='--', lw=1)
    ax.set(title='Simulation speed under motion', xlabel='Requested physics rate (steps/s)',
           ylabel='Simulated time / wall time', ylim=(0, 105), xlim=(0, largest * 1.04))
    ax.yaxis.set_major_formatter(PercentFormatter(100))

    # One symbol per raw trial: repeats stay visible; no interpolated maximum.
    ax = axs[1, 0]
    x = np.linspace(min(r['requested_hz'] for r in rows), min(largest, 550), 200)
    ax.plot(x, 1000 / x, color='#94A3B8', lw=1.5, linestyle='--', label='Physics dt budget')
    for metric, color, marker, label in [('physics_mean_us', '#3478AD', 'o', 'Mean'),
                                         ('physics_p99_us', '#CB8B22', '^', '99th percentile')]:
        selected = [r for r in motion if r['requested_hz'] <= 550 and r[metric] is not None]
        for traced in (False, True):
            samples = [r for r in selected if r['traced'] == traced]
            ax.scatter([r['requested_hz'] for r in samples], [r[metric]/1000 for r in samples],
                       s=27, edgecolors=color, facecolors='none' if traced else color,
                       marker=marker, label=label if not traced else None, zorder=3)
    ax.set(title='Time spent in each physics call', xlabel='Requested physics rate (steps/s)',
           ylabel='Milliseconds (JNI + physics + decode)', xlim=(min(x)-15, max(x)+15))
    ax.legend(frameon=False, fontsize=8, loc='upper right')

    ax = axs[1, 1]
    # Aggregate repeats at each requested rate as median components, only untraced motion.
    untraced = [r for r in motion if not r['traced']]
    rates = sorted({r['requested_hz'] for r in untraced})
    bottom = np.zeros(len(rates))
    for name, color, label in [('physics', '#3478AD', 'Physics'), ('base', '#5DAFA3', 'Base updates'), ('ik', '#CB8B22', 'IK')]:
        values = []
        for rate in rates:
            samples = [100 * (r[name+'_mean_us'] or 0) * r[name+'_count'] / 1e6 / r['wall_seconds']
                       for r in untraced if r['requested_hz'] == rate]
            values.append(float(np.median(samples)))
        ax.bar([str(r) for r in rates], values, bottom=bottom, color=color, label=label, zorder=3)
        bottom += values
    ax.axhline(100, color='#94A3B8', linestyle='--', lw=1)
    ax.set(title='Simulation worker · median untraced components', xlabel='Requested physics rate (steps/s)',
           ylabel='Timed call duration / wall time', ylim=(0, 112))
    ax.yaxis.set_major_formatter(PercentFormatter(100))
    ax.legend(frameon=False, fontsize=8, loc='upper left', ncol=3)

    fig.legend(handles=[Line2D([], [], marker='o', color=colors['pass'], ls='', label='Meets native timing gates'),
                        Line2D([], [], marker='o', color=colors['fail'], ls='', label='Misses at least one native timing gate'),
                        Line2D([], [], marker='s', color=colors['rest'], ls='', label='Rest control'),
                        Line2D([], [], marker='o', markerfacecolor='none', color='#334155', ls='', label='Perfetto repeat')],
               loc='lower left', bbox_to_anchor=(.06, .065), ncol=2, frameon=False, fontsize=9)
    fig.text(.07, .031, 'Native gates: ≥99% real time, ≤0.1% discarded time, physics p99 ≤ dt, ≥88.2 rendered frames/s. Compositor trace checks are separate.',
             fontsize=8.5, color='#475569')
    if failed_count:
        fig.text(.07, .012, f'{failed_count} failed trial(s) retained in CSV/JSON, excluded from these timing plots.', fontsize=8)
    for extension in ('png', 'svg', 'pdf'):
        fig.savefig(output / ('physics-rate-sweep.' + extension), dpi=180, facecolor='white')
    plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.8))
    fig.subplots_adjust(top=.78, bottom=.2, wspace=.25)
    fig.suptitle('Rendering and device health during the sweep', x=.075, ha='left', y=.97,
                 fontsize=20, fontweight='bold')
    for name, marker, color, label in [('xr_cpu_p99_us', 'o', '#3478AD', 'XR CPU p99'),
                                       ('gpu_p99_us', '^', '#17806D', 'App GPU p99')]:
        for traced in (False, True):
            samples = [r for r in rows if r[name] is not None and r['traced'] == traced
                       and (name != 'gpu_p99_us' or r['gpu_supported'] and r['gpu_disjoint_count'] == 0)]
            axs[0].scatter([r['requested_hz'] for r in samples], [r[name]/1000 for r in samples],
                           edgecolors=color, facecolors='none' if traced else color,
                           marker=marker, s=32, label=label if not traced else None)
    axs[0].axhline(1000/90, color='#94A3B8', ls='--', label='90 Hz frame period')
    axs[0].set(xlabel='Requested physics rate (steps/s)', ylabel='Milliseconds', title='CPU and GPU are separate, parallel work')
    axs[0].legend(frameon=False, fontsize=8)
    for metric, label, color in [('cpu_max_celsius', 'Hottest CPU sensor', '#D45C3D'),
                                  ('gpu_max_celsius', 'Hottest GPU sensor', '#3478AD'),
                                  ('surface_celsius', 'Surface estimate', '#17806D')]:
        values = [r['after_'+metric] if r['after_'+metric] is not None else float('nan') for r in rows]
        axs[1].plot(range(1, len(rows)+1), values, marker='.', color=color, lw=1, label=label)
    axs[1].set(xlabel='Trial number in chronological order', ylabel='Temperature (°C)', title='After-trial device sensors')
    axs[1].legend(frameon=False, fontsize=8)
    for ax in axs:
        ax.grid(axis='y')
    fig.text(.075, .065, 'Hollow points: Perfetto repeats. Sensor snapshots with passthrough active; not a long-duration thermal qualification.', fontsize=9)
    for extension in ('png', 'svg', 'pdf'):
        fig.savefig(output / ('rendering-health.' + extension), dpi=180, facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, evidence = collect(args.root)
    with (args.output / 'trials.csv').open('w', newline='', encoding='utf-8') as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.output / 'trials.json').write_text(json.dumps(dict(rows=rows, evidence=evidence), indent=2), encoding='utf-8')
    plot(rows, args.output)
    print(json.dumps(dict(trials=len(rows), output=str(args.output.resolve()))))


if __name__ == '__main__':
    main()
