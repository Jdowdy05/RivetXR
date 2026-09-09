"""Host-only tests for evidence boundaries in the Perfetto analyzer."""

import unittest
import sqlite3
import tempfile
from pathlib import Path
import contextlib
import io
import json
from unittest.mock import patch

from tools.performance import analyze_perfetto

from tools.performance.analyze_perfetto import (
    analyze, capture_interval, choose_process, coverage, pair_frames, quality_issues, sampled_series, stats,
)


class PerfettoEvidenceTests(unittest.TestCase):
    def test_missing_samples_remain_unknown(self):
        self.assertIsNone(stats([])["mean"])
        self.assertIsNone(coverage([], (0, 100))["interval_fraction"])
        self.assertIsNone(sampled_series([], (0, 100))["sample_rate_hz"])

    def test_process_selection_uses_unique_measurement_owner(self):
        processes = [{"upid": 2}, {"upid": 8}]
        self.assertEqual(choose_process(processes, [{"upid": 8, "dur": 5}]), 8)
        self.assertIsNone(choose_process(processes, []))
        self.assertIsNone(choose_process(processes, [
            {"upid": 8, "dur": 5}, {"upid": 2, "dur": 5}]))

    def test_coverage_does_not_extrapolate_missing_tail(self):
        result = coverage([{"ts": 10, "dur": 5}, {"ts": 50, "dur": 5}], (0, 100))
        self.assertEqual(result["interval_fraction"], .45)
        self.assertEqual(result["end_ts"], 55)

    def test_capture_window_excludes_historical_process_start_timestamps(self):
        metadata = [{"name": "tracing_started_ns", "int_value": 100},
                    {"name": "tracing_disabled_ns", "int_value": 200}]
        self.assertEqual(capture_interval(metadata, {"start_ts": 0, "end_ts": 200}), (100, 200))

    def test_buffer_discards_count_even_when_severity_info(self):
        rows = [{"name": "traced_buf_chunks_discarded", "value": 2, "severity": "info"},
                {"name": "traced_buf_bytes_written", "value": 123, "severity": "info"}]
        self.assertEqual(quality_issues(rows), rows[:1])

    def test_frame_pairs_require_matching_thread_and_frame_id(self):
        rows = [
            {"name": "xrBeginFrame : 42", "utid": 1, "ts": 10, "dur": 1},
            {"name": "!xrEndFrame : 42", "utid": 2, "ts": 13, "dur": 1},
            {"name": "!xrEndFrame : 42", "utid": 1, "ts": 14, "dur": 2},
            {"name": "xrBeginFrame : 43", "utid": 1, "ts": 20, "dur": 1},
            {"name": "!xrEndFrame : 43", "utid": 1, "ts": 22, "dur": -1},
        ]
        paired, unmatched = pair_frames(rows)
        self.assertEqual(len(paired), 1)
        self.assertEqual(paired[0]["dur"], 6)
        self.assertEqual(unmatched, 1)

    def test_repeated_runtime_values_are_not_independent_timings(self):
        rows = [{"ts": 0, "value": 2}, {"ts": 1_000_000_000, "value": 2},
                {"ts": 2_000_000_000, "value": 4}]
        result = sampled_series(rows, (0, 3_000_000_000))
        self.assertEqual(result["distinct_values"], 2)
        self.assertEqual(result["value_transitions"], 1)
        self.assertEqual(result["sample_rate_hz"], 1)
        self.assertEqual(result["sample_interval_ms"]["p99"], 1000)


class FullAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.db = sqlite3.connect(":memory:")
        self.addCleanup(self.db.close)
        self.db.executescript("""
            CREATE TABLE analysis_stats(name,idx,severity,value);
            CREATE TABLE analysis_trace_bounds(start_ts,end_ts);
            INSERT INTO analysis_trace_bounds VALUES(0,40000000000);
            CREATE TABLE metadata(name,int_value,str_value);
            CREATE TABLE process(upid,pid,name,start_ts,end_ts);
            INSERT INTO process VALUES(7,42,'com.questnewton',NULL,NULL);
            CREATE TABLE thread(utid,upid);
            INSERT INTO thread VALUES(1,7),(2,7);
            CREATE TABLE thread_track(id,utid);
            INSERT INTO thread_track VALUES(1,1),(2,2);
            CREATE TABLE process_track(id,upid,name);
            INSERT INTO process_track VALUES(3,7,'AppInfo');
            CREATE TABLE slice(id,name,ts,dur,thread_dur,track_id,arg_set_id);
            INSERT INTO slice VALUES(0,'QuestFull.BenchmarkMeasure',1000000000,30000000000,NULL,2,NULL);
            INSERT INTO slice VALUES(1,'QuestFull.BenchmarkStep',1100000000,1000000,NULL,2,NULL);
            INSERT INTO slice VALUES(2,'QuestFull.BenchmarkStep',30900000000,1000000,NULL,2,NULL);
            CREATE TABLE sched_slice(utid,ts,dur);
            INSERT INTO sched_slice VALUES(1,0,40000000000),(2,0,40000000000);
            CREATE TABLE args(arg_set_id,key,real_value,int_value);
            CREATE TABLE counter_track(id,name,unit,type);
            CREATE TABLE process_counter_track(id,upid);
            CREATE TABLE counter(track_id,ts,value);
            CREATE TABLE gpu_slice(id);
            CREATE TABLE clock_snapshot(ts,clock_value,clock_name);
            INSERT INTO clock_snapshot VALUES(0,0,'MONOTONIC');
        """)
        for i in range(3000):
            ts = 1_000_000_000 + i * 10_000_000
            for name, offset, dur in (("QuestNewton.FrameCPU", 0, 1_000_000),
                                     (f"xrBeginFrame : {i}", 0, 100),
                                     (f"!xrEndFrame : {i}", 1_000_000, 100)):
                self.db.execute("INSERT INTO slice VALUES(?,?,?,?,NULL,1,NULL)", (10 + i * 4, name, ts + offset, dur))
            self.db.execute("INSERT INTO slice VALUES(?, 'AppInfo', ?, 0, NULL, 3, ?)", (13 + i * 4, ts, i))
            for key, value in (("debug.frameIndex", i), ("debug.staleFramesPerSecond", 0), ("debug.repeatedFrameCountMax", 0)):
                self.db.execute("INSERT INTO args VALUES(?,?,NULL,?)", (i, key, value))
        self.native = dict(run_id="test", pid=42, phase="complete", requested_seconds=30, wall_seconds=30,
            successful_steps=2, measure_started_monotonic_s=1, measure_ended_monotonic_s=31)

    def test_complete_interval_has_eligible_evidence_and_exact_native_pairing(self):
        result, queries = analyze(self.db, self.native)
        self.assertTrue(result["strong_xr_evidence_eligible"], result["xr_evidence_limitations"])
        self.assertEqual(result["observed_measured_steps"], 2)
        self.assertTrue(result["native_consistency"]["endpoints_match"])
        self.assertGreater(len(queries), 5)

    def test_wrong_native_identity_timing_and_count_block_eligibility(self):
        self.native.update(measure_started_monotonic_s=2, successful_steps=3)
        result, _ = analyze(self.db, self.native)
        self.assertFalse(result["strong_xr_evidence_eligible"])
        self.assertFalse(result["native_consistency"]["endpoints_match"])
        self.assertFalse(result["native_consistency"]["step_count_matches"])

    def test_native_pid_must_match_the_trace_measurement_process(self):
        for pid in (999, None, True, "42"):
            with self.subTest(pid=pid):
                self.native['pid'] = pid
                result, _ = analyze(self.db, self.native)
                self.assertFalse(result['strong_xr_evidence_eligible'])

    def test_matched_pid_is_recorded_in_consistency_evidence(self):
        result, _ = analyze(self.db, self.native)
        self.assertIs(result['native_consistency']['pid_matches'], True)
        self.assertEqual(result['native_consistency']['observed_pid'], 42)

    def test_explicit_native_file_cannot_turn_into_trace_only_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'native.json'
            for raw in ('null', '[]', 'true', '42', '"record"'):
                path.write_text(raw, encoding='utf-8')
                with self.subTest(raw=raw), self.assertRaises(ValueError):
                    analyze_perfetto.load_native_result(path)
            path.write_text('{"pid":42}', encoding='utf-8')
            self.assertEqual(analyze_perfetto.load_native_result(path), {'pid': 42})
        self.assertIsNone(analyze_perfetto.load_native_result(None))

    def test_cli_rejects_null_native_before_processing_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            native, output = root / 'native.json', root / 'analysis.json'
            native.write_text('null', encoding='utf-8')
            with patch.object(analyze_perfetto.subprocess, 'run') as processor, contextlib.redirect_stdout(io.StringIO()):
                code = analyze_perfetto.main(['--trace', str(root / 'trace.pftrace'),
                                             '--output', str(output), '--native-result', str(native)])
            self.assertEqual(code, 1)
            processor.assert_not_called()
            result = json.loads(output.read_text(encoding='utf-8'))
            self.assertFalse(result['strong_xr_evidence_eligible'])
            self.assertIn('JSON object', result['analysis_error'])

    def test_missing_stale_is_null_and_blocks_eligibility(self):
        self.db.execute("DELETE FROM args WHERE key='debug.staleFramesPerSecond'")
        result, _ = analyze(self.db, self.native)
        self.assertFalse(result["strong_xr_evidence_eligible"])
        self.assertIsNone(result["stale_evidence"]["appinfo_mean_fraction_of_90hz"])

    def test_short_or_failed_trace_never_substitutes_probe_steps(self):
        self.db.execute("UPDATE slice SET dur=-1 WHERE name='QuestFull.BenchmarkMeasure'")
        result, _ = analyze(self.db, self.native)
        self.assertFalse(result["strong_xr_evidence_eligible"])
        self.assertIsNone(result["observed_measured_steps"])
        self.assertIsNone(result["native_consistency"]["duration_matches"])


if __name__ == "__main__":
    unittest.main()
