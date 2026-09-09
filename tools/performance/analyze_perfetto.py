"""Analyze Quest benchmark Perfetto evidence without device operations.

Example: python -m tools.performance.analyze_perfetto --trace trial.pftrace
    --output trial-perfetto.json --native-result trial-native.json

The authoritative input is the trace. SQLite, import log and exact SQL are saved
alongside the JSON. Statistics use nearest-rank percentiles and unweighted sample
means. Runtime gpuTime is sampled telemetry, not independent GPU timestamps.
No CPU/GPU sums, theoretical maximum physics rates or overall passes are derived.
"""

from __future__ import annotations

import argparse
import bisect
import collections
import csv
import io
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import subprocess


MEASURE = "QuestFull.BenchmarkMeasure"
STEP = "QuestFull.BenchmarkStep"
FRAME = "QuestNewton.FrameCPU"
META_STATS_URL = "https://developers.meta.com/horizon/llmstxt/documentation/native/android/ts-ovrstats.md"
# The bundled processor exports only its predefined tables, not arbitrary SQL
# tables or its virtual stats/trace_bounds tables. Preserve these via query CSV.
EXPORT_SQL = """SELECT 'stat' AS row_type,name,idx,severity,value,NULL AS start_ts,NULL AS end_ts FROM stats
UNION ALL SELECT 'bounds',NULL,NULL,NULL,NULL,start_ts,end_ts FROM trace_bounds;"""


def stats(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    n = len(values)
    result = dict(n=n, min=None, mean=None, p50=None, p95=None, p99=None, max=None)
    if n:
        result.update(min=values[0], mean=sum(values) / n, max=values[-1])
        for name, p in (("p50", .5), ("p95", .95), ("p99", .99)):
            result[name] = values[math.ceil(n * p) - 1]
    return result


def coverage(rows, interval=None):
    start = min((r["ts"] for r in rows), default=None)
    end = max((r["ts"] + max(r.get("dur", 0) or 0, 0) for r in rows), default=None)
    fraction = None
    if start is not None and interval and interval[1] > interval[0]:
        fraction = max(0, min(end, interval[1]) - max(start, interval[0])) / (interval[1] - interval[0])
    return dict(start_ts=start, end_ts=end, span_s=(end - start) / 1e9 if start is not None else None,
                interval_fraction=fraction)


def sampled_series(rows, interval):
    rows = sorted(rows, key=lambda r: r["ts"])
    values = [r["value"] for r in rows]
    deltas = [(b["ts"] - a["ts"]) / 1e6 for a, b in zip(rows, rows[1:])]
    span = coverage(rows, interval)
    transitions = sum(a != b for a, b in zip(values, values[1:]))
    return dict(stats=stats(values), coverage=span, sample_interval_ms=stats(deltas),
                sample_rate_hz=(len(rows) - 1) / span["span_s"] if span["span_s"] else None,
                distinct_values=len(set(v for v in values if v is not None)),
                value_transitions=transitions,
                transition_rate_hz=transitions / span["span_s"] if span["span_s"] else None,
                adjacent_equal_values=sum(a == b for a, b in zip(values, values[1:])),
                nonzero_samples=sum(v not in (0, None) for v in values),
                first=values[0] if values else None, last=values[-1] if values else None)


def choose_process(processes, measures):
    owners = {m["upid"] for m in measures if m["dur"] > 0}
    if len(owners) == 1:
        return next(iter(owners))
    if len(processes) == 1:
        return processes[0]["upid"]
    return None


def capture_interval(metadata, bounds):
    recorded = {r["name"]: r["int_value"] for r in metadata}
    start, end = recorded.get("tracing_started_ns"), recorded.get("tracing_disabled_ns")
    # trace_bounds can include historical process-start timestamps far before
    # tracing was enabled. Keep those raw bounds, but do not use them as exposure.
    if start is not None and end is not None and end > start:
        return start, end
    return bounds["start_ts"], bounds["end_ts"]


def quality_issues(rows):
    loss = re.compile(r"discard|patches_failed|overrun|overwrite|dropped|data_loss", re.I)
    return [r for r in rows if r["value"] and
            (r["severity"] in ("error", "data_loss") or loss.search(r["name"]))]


def pair_frames(rows):
    """Pair by process-scoped utid and frame suffix; reject duplicates/partials."""
    beginnings, endings = collections.defaultdict(list), collections.defaultdict(list)
    for r in rows:
        match = re.fullmatch(r"(!?xr(?:Begin|End)Frame)\s*:\s*(\d+)", r["name"])
        if not match:
            continue
        key = (r["utid"], match[2])
        (beginnings if match[1] == "xrBeginFrame" else endings)[key].append(r)
    pairs = []
    for key, starts in beginnings.items():
        ends = endings.get(key, [])
        if len(starts) == len(ends) == 1:
            a, b = starts[0], ends[0]
            if a["dur"] >= 0 and b["dur"] >= 0 and b["ts"] >= a["ts"]:
                pairs.append(dict(ts=a["ts"], dur=b["ts"] + b["dur"] - a["ts"],
                                  utid=a["utid"], frame_index=int(key[1])))
    return sorted(pairs, key=lambda r: r["ts"]), sum(map(len, beginnings.values())) - len(pairs)


class Queries:
    def __init__(self, connection):
        self.connection = connection
        connection.row_factory = sqlite3.Row
        self.log = []

    def __call__(self, label, sql, parameters=()):
        entry = dict(label=label, sql=sql, parameters=list(parameters))
        self.log.append(entry)
        return [dict(r) for r in self.connection.execute(sql, parameters)]


def cadence(rows, interval, refresh_hz=90):
    rows = sorted(rows, key=lambda r: r["ts"])
    differences = [(b["ts"] - a["ts"]) / 1e6 for a, b in zip(rows, rows[1:])]
    span = coverage(rows, interval)
    return dict(n=len(rows), coverage=span, interval_ms=stats(differences),
                rate_hz=(len(rows) - 1) * 1e9 / (rows[-1]["ts"] - rows[0]["ts"]) if len(rows) > 1 and rows[-1]["ts"] > rows[0]["ts"] else None,
                over_1point5_display_intervals=sum(v > 1500 / refresh_hz for v in differences),
                display_reference_hz=refresh_hz,
                note="Start cadence and large gaps; intervals above one period alone are not counted stale frames.")


def inside(rows, interval, complete=False):
    a, b = interval
    return [r for r in rows if a <= r["ts"] < b and
            (not complete or (r.get("dur", 0) >= 0 and r["ts"] + r.get("dur", 0) <= b))]


def native_consistency(native, measure, q, step_count, observed_pid=None):
    if native is None:
        return None
    result = {key: native.get(key) for key in (
        "run_id", "pid", "phase", "requested_physics_hz", "requested_seconds", "wall_seconds",
        "successful_steps", "finite", "failure", "measure_started_monotonic_s", "measure_ended_monotonic_s")}
    pid = native.get('pid')
    result.update(observed_pid=observed_pid,
                  pid_matches=type(pid) is int and pid > 0 and type(observed_pid) is int and pid == observed_pid,
                  measurement_duration_delta_s=None, duration_matches=None, endpoint_delta_s=None,
                  endpoints_match=None, observed_step_count=step_count,
                  step_count_matches=step_count == native["successful_steps"] if "successful_steps" in native and measure else None,
                  requested_duration_reached=None)
    if not measure:
        return result
    trace_seconds = measure["dur"] / 1e9
    wall = native.get("wall_seconds")
    if isinstance(wall, (int, float)):
        delta = trace_seconds - wall
        result.update(measurement_duration_delta_s=delta, duration_matches=abs(delta) <= .02)
    requested = native.get("requested_seconds")
    if isinstance(requested, (int, float)):
        result["requested_duration_reached"] = trace_seconds >= requested - .02
    clocks = q("native monotonic clock mapping", "SELECT ts,clock_value FROM clock_snapshot WHERE clock_name='MONOTONIC' ORDER BY ts")
    endpoints = [native.get("measure_started_monotonic_s"), native.get("measure_ended_monotonic_s")]
    if clocks and all(isinstance(x, (int, float)) and x > 0 for x in endpoints):
        deltas = []
        for native_s, trace_ns in zip(endpoints, (measure["ts"], measure["ts"] + measure["dur"])):
            clock = min(clocks, key=lambda c: abs(c["clock_value"] - native_s * 1e9))
            mapped = native_s * 1e9 + clock["ts"] - clock["clock_value"]
            deltas.append((trace_ns - mapped) / 1e9)
        result.update(endpoint_delta_s=deltas, endpoints_match=all(abs(x) <= .02 for x in deltas))
    result["tolerance_seconds"] = .02
    return result


def analyze(connection, native=None):
    q = Queries(connection)
    quality = q("nonzero quality and buffer stats", "SELECT name,idx,severity,value FROM analysis_stats WHERE value<>0 AND (severity IN ('error','data_loss') OR name LIKE 'traced_buf%' OR name LIKE 'ftrace%')")
    issues = quality_issues(quality)
    bounds = q("trace bounds", "SELECT start_ts,end_ts FROM analysis_trace_bounds")[0]
    metadata = q("trace metadata", "SELECT name,int_value,str_value FROM metadata WHERE name IN ('tracing_started_ns','tracing_disabled_ns','all_data_source_started_ns','trace_time_clock_id','trace_config_pbtxt','trace_uuid')")
    processes = q("application process identities", "SELECT upid,pid,name,start_ts,end_ts FROM process WHERE name='com.questnewton' ORDER BY upid")
    base = "SELECT s.id,s.name,s.ts,s.dur,s.thread_dur,tt.utid,t.upid FROM slice s JOIN thread_track tt ON tt.id=s.track_id JOIN thread t USING(utid) "
    measures = q("measurement markers all app lifetimes", base + "JOIN process p USING(upid) WHERE p.name='com.questnewton' AND s.name=? ORDER BY s.ts", (MEASURE,))
    upid = choose_process(processes, measures)
    result = dict(schema_version=1, source=META_STATS_URL,
                  trace_quality=dict(low_loss=not issues, nonzero_issues=issues, nonzero_stats=quality),
                  trace_bounds=bounds, metadata=metadata, processes=processes, selected_upid=upid,
                  measurement_markers=measures, measurement=None,
                  strong_xr_evidence_eligible=False, xr_evidence_limitations=[])
    if upid is None:
        result["xr_evidence_limitations"].append("Application process is missing or ambiguous across lifetimes.")
        return result, q.log
    selected_measures = [m for m in measures if m["upid"] == upid and m["dur"] > 0]
    measure = selected_measures[0] if len(selected_measures) == 1 else None
    captured = capture_interval(metadata, bounds)
    result["capture_interval"] = dict(start_ts=captured[0], end_ts=captured[1], seconds=(captured[1] - captured[0]) / 1e9)
    interval = (measure["ts"], measure["ts"] + measure["dur"]) if measure else captured
    result["analysis_interval"] = dict(start_ts=interval[0], end_ts=interval[1], seconds=(interval[1] - interval[0]) / 1e9,
                                       scope="complete BenchmarkMeasure" if measure else "retained observations within capture window; benchmark interval unavailable")
    result["measurement"] = measure
    custom = q("app custom markers", base + "WHERE t.upid=? AND (s.name LIKE 'QuestFull.%' OR s.name LIKE 'QuestNewton.%') ORDER BY s.ts", (upid,))
    result["retained_custom_markers"] = {name: dict(n=len(rows), coverage=coverage(rows), incomplete=sum(r["dur"] < 0 for r in rows))
        for name, rows in _group(custom, "name").items()}
    selected = inside(custom, interval, complete=True)
    selected = [r for r in selected if r["name"] not in (MEASURE, "QuestFull.BenchmarkWarmup")]
    schedules = {}
    for utid in {r["utid"] for r in selected}:
        rows = q("app scheduling utid " + str(utid), "SELECT ts,dur FROM sched_slice WHERE utid=? AND dur>=0 ORDER BY ts", (utid,))
        schedules[utid] = (rows, [r["ts"] for r in rows])
    result["scheduling_coverage"] = {str(utid): coverage(rows, interval) for utid, (rows, _) in schedules.items()}
    costs = {}
    for name, rows in _group(selected, "name").items():
        running, not_running = [], []
        for r in rows:
            sched, starts = schedules[r["utid"]]
            if not sched or r["ts"] < sched[0]["ts"] or r["ts"] + r["dur"] > sched[-1]["ts"] + sched[-1]["dur"]:
                continue
            j = max(0, bisect.bisect_right(starts, r["ts"]) - 1)
            total = 0
            while j < len(sched) and sched[j]["ts"] < r["ts"] + r["dur"]:
                s = sched[j]
                total += max(0, min(r["ts"] + r["dur"], s["ts"] + s["dur"]) - max(r["ts"], s["ts"]))
                j += 1
            running.append(total / 1e6)
            not_running.append((r["dur"] - total) / 1e6)
        costs[name] = dict(n=len(rows), coverage=coverage(rows, interval),
                           wall_ms=stats(r["dur"] / 1e6 for r in rows), scheduled_cpu_ms=stats(running),
                           not_running_ms=stats(not_running), scheduled_edge_samples_excluded=len(rows) - len(running),
                           cadence=cadence(rows, interval))
    result["cpu_costs"] = costs
    result["observed_measured_steps"] = costs.get(STEP, {}).get("n", 0) if measure else None
    observed_pid = next((process['pid'] for process in processes if process['upid'] == upid), None)
    result["native_consistency"] = native_consistency(native, measure, q, result["observed_measured_steps"], observed_pid)
    frames = q("app xr begin and end markers", base + "WHERE t.upid=? AND (s.name LIKE 'xrBeginFrame : %' OR s.name LIKE '!xrEndFrame : %') ORDER BY s.ts", (upid,))
    pairs, unmatched = pair_frames(frames)
    measured_pairs = inside(pairs, interval, complete=True)
    result["xr_frames"] = dict(paired=cadence(measured_pairs, interval),
                                retained_pair_coverage=coverage(pairs, interval), unmatched_begins_whole_trace=unmatched,
                                cpu_begin_to_end_ms=stats(r["dur"] / 1e6 for r in measured_pairs))
    app = q("process-bound AppInfo numeric fields", "SELECT s.id,s.ts,s.dur,a.key,COALESCE(a.real_value,a.int_value) AS value FROM slice s JOIN process_track pt ON pt.id=s.track_id JOIN args a USING(arg_set_id) WHERE pt.upid=? AND pt.name='AppInfo' AND a.key LIKE 'debug.%' ORDER BY s.ts,a.key", (upid,))
    result["appinfo_retained_coverage"] = coverage(app, interval)
    app_selected = inside(app, interval)
    runtime = {key: sampled_series(rows, interval) for key, rows in _group(app_selected, "key").items()}
    result["runtime_appinfo"] = runtime
    frame_indices = [r["value"] for r in app_selected if r["key"] == "debug.frameIndex" and r["value"] is not None]
    result["appinfo_frame_indices"] = dict(n=len(frame_indices), unique=len(set(frame_indices)),
        nonconsecutive_transitions=sum(b - a != 1 for a, b in zip(frame_indices, frame_indices[1:])),
        first=frame_indices[0] if frame_indices else None, last=frame_indices[-1] if frame_indices else None)
    tracks = q("app and system counter inventory", "SELECT ct.id,ct.name,ct.unit,ct.type,pt.upid FROM counter_track ct LEFT JOIN process_counter_track pt ON ct.id=pt.id WHERE pt.upid=? OR (pt.upid IS NULL AND ct.type<>'gpu_counter_track') ORDER BY ct.id", (upid,))
    # Select low-volume named telemetry; hardware counters can duplicate names and
    # are device-wide. Do not sweep the high-volume GPU counter table implicitly.
    selector = re.compile(r"stale|app_|display_refresh|frequency|available_memory|cpu_level|gpu_level|cpu_util_\d|gpu_util$|timewarp_gpu|guardian_gpu|screen_tears|thermal|temperature|battery|pss|^mem\.|^cpufreq$|^devfreq", re.I)
    counters = {}
    for tr in tracks:
        if not selector.search(tr["name"]):
            continue
        rows = q("counter " + str(tr["id"]), "SELECT ts,value FROM counter WHERE track_id=? AND ts>=? AND ts<? ORDER BY ts", (tr["id"], *interval))
        if rows:
            counters[str(tr["id"])] = dict(**tr, **sampled_series(rows, interval), scope="application" if tr["upid"] == upid else "device-wide")
    result["counters"] = counters
    stale = runtime.get("debug.staleFramesPerSecond")
    stale_counter = [v for v in counters.values() if v["upid"] == upid and v["name"] == "stale_frames_per_second"]
    repeat = runtime.get("debug.repeatedFrameCountMax")
    result["stale_evidence"] = dict(
        appinfo_stale_per_second=stale, counter_stale_per_second=stale_counter or None,
        appinfo_mean_fraction_of_90hz=stale["stats"]["mean"] / 90 if stale and stale["stats"]["mean"] is not None else None,
        counter_mean_fraction_of_90hz=stale_counter[0]["stats"]["mean"] / 90 if len(stale_counter) == 1 else None,
        rolling_repeated_frame_max=repeat,
        units="staleFramesPerSecond and stale_frames_per_second: frames/second; divide by 90 frames/second for a fraction, multiply by 100 for percent. Never sum repeated rolling samples as missed frames.",
        repeated_max_note="Rolling/reported maximum, not an additive independent frame count; a nonzero value can reflect an earlier interval.")
    raw_gpu = q("raw GPU slice availability", "SELECT COUNT(*) AS n FROM gpu_slice")[0]["n"]
    result["gpu_evidence"] = dict(raw_gpu_slice_count=raw_gpu,
        app_raw_gpu_timing_ms=None, runtime_sampled_gpu_time=runtime.get("debug.gpuTime"),
        app_gpu_ms_counters=[v for v in counters.values() if v["name"] == "app_gpu_ms" and v["upid"] == upid] or None,
        note="gpuTime is repeated sampled runtime telemetry; app_gpu_ms names milliseconds. AppInfo GPU unit must be cross-checked against app_gpu_ms. SliceGpu0/1 are not attributed to app GPU. No independent app pass timings reconstructed.")
    result["missing_telemetry"] = {name: None for name, pattern in (
        ("pss", r"pss"), ("temperature", r"temperature"), ("thermal_status", r"thermal"), ("battery", r"battery"))
        if not any(re.search(pattern, t["name"], re.I) for t in counters.values())}
    reasons = result["xr_evidence_limitations"]
    if not measure:
        reasons.append("A unique complete BenchmarkMeasure interval is unavailable.")
    if issues:
        reasons.append("Nonzero trace loss/error statistics; retained spans do not prove complete capture.")
    streams = dict(paired_xr=coverage(measured_pairs, interval), appinfo=coverage(app_selected, interval),
                   native_frame=costs.get(FRAME, {}).get("coverage", coverage([], interval)))
    result["measurement_stream_coverage"] = streams
    for name, span in streams.items():
        if span["interval_fraction"] is None or span["interval_fraction"] < .98:
            reasons.append(name + " coverage is missing or below 98% of the analyzed interval.")
    if not stale or not repeat:
        reasons.append("Process-bound stale or repeated-frame telemetry is unavailable.")
    if not frame_indices:
        reasons.append("AppInfo frame-index continuity is unavailable.")
    elif result["appinfo_frame_indices"]["nonconsecutive_transitions"]:
        reasons.append("AppInfo frame indices have nonconsecutive transitions.")
    consistency = result["native_consistency"]
    if consistency:
        for field in ("pid_matches", "duration_matches", "endpoints_match", "step_count_matches", "requested_duration_reached"):
            if consistency[field] is not True:
                reasons.append("Native/trace consistency unavailable or failed: " + field)
        if native.get("phase") != "complete":
            reasons.append("Native trial did not complete.")
    result["strong_xr_evidence_eligible"] = not reasons
    result["interpretation"] = "Eligibility is an evidence-quality check, not a useful physics rate or XR pass. Inspect stale/repeat values, cadence, native completion and device health together. CPU and GPU execute in parallel; never sum their durations to derive FPS or stale frames. Coverage is first-to-last span, not proof against interior loss."
    return result, q.log


def _group(rows, key):
    grouped = collections.defaultdict(list)
    for row in rows:
        grouped[row[key]].append(row)
    return dict(grouped)


def load_native_result(path):
    if path is None:
        return None
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError("Explicit native result must be a JSON object")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--native-result", type=Path)
    default = Path(os.environ.get("APPDATA", "")) / "metavr/tools/perfetto/trace_processor_shell.exe"
    parser.add_argument("--processor", type=Path, default=default)
    args = parser.parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sqlite_path = args.output.with_suffix(".sqlite")
    sql_path = args.output.with_suffix(".export.sql")
    queries_path = args.output.with_suffix(".queries.json")
    log_path = args.output.with_suffix(".import.log")
    sql_path.write_text(EXPORT_SQL, encoding="utf-8")
    command = [str(args.processor), "--no-ftrace-raw", "-q", str(sql_path), "-e", str(sqlite_path), str(args.trace)]
    try:
        native = load_native_result(args.native_result)
        # Always reimport: a changed trace/native result must never reuse stale data.
        if sqlite_path.exists():
            sqlite_path.unlink()
        with log_path.open("w", encoding="utf-8") as log:
            imported = subprocess.run(command, stdout=subprocess.PIPE, stderr=log, check=True, text=True, encoding="utf-8")
        args.output.with_suffix(".quality.csv").write_text(imported.stdout, encoding="utf-8")
        with sqlite3.connect(sqlite_path) as connection:
            connection.execute("CREATE TABLE analysis_stats(name TEXT,idx INTEGER,severity TEXT,value INTEGER)")
            connection.execute("CREATE TABLE analysis_trace_bounds(start_ts INTEGER,end_ts INTEGER)")
            for row in csv.DictReader(io.StringIO(imported.stdout.strip())):
                if row["row_type"] == "stat":
                    connection.execute("INSERT INTO analysis_stats VALUES (?,?,?,?)",
                        (row["name"], int(row["idx"]) if row["idx"] != "[NULL]" else None, row["severity"], int(row["value"])))
                else:
                    connection.execute("INSERT INTO analysis_trace_bounds VALUES (?,?)", (int(row["start_ts"]), int(row["end_ts"])))
            connection.commit()
            result, queries = analyze(connection, native)
        result.update(trace=str(args.trace.resolve()), native_result=str(args.native_result.resolve()) if args.native_result else None,
                      sql_queries_file=str(queries_path.resolve()), export_command=command)
        queries_path.write_text(json.dumps(queries, indent=2), encoding="utf-8")
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps({"output": str(args.output), "low_loss": result["trace_quality"]["low_loss"],
                          "strong_xr_evidence_eligible": result["strong_xr_evidence_eligible"],
                          "measurement_seconds": result["measurement"]["dur"] / 1e9 if result["measurement"] else None,
                          "limitations": result["xr_evidence_limitations"]}))
        return 0
    except (OSError, ValueError, sqlite3.Error, subprocess.CalledProcessError) as error:
        args.output.write_text(json.dumps(dict(schema_version=1, trace=str(args.trace.resolve()),
            analysis_error=str(error), strong_xr_evidence_eligible=False, import_log=str(log_path.resolve())), indent=2), encoding="utf-8")
        print(f"Analysis failed; see {args.output} and {log_path}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
