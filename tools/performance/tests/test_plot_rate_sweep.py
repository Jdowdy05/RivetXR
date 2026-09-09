"""CSV regressions for unsuccessful and successful native benchmark windows."""

import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools.performance import plot_rate_sweep


def trial(run_id, successful, started):
    steps = 6000 if successful else 0
    timing = dict(available=successful, count=steps,
                  mean_us=1000 if successful else None,
                  p95_us=1800 if successful else None,
                  p99_us=2000 if successful else None,
                  max_us=2200 if successful else None, over_budget_count=0)
    native = dict(run_id=run_id, workload="motion", requested_physics_hz=200,
                  phase="complete" if successful else "failed", finite=successful,
                  passed=successful, failure="" if successful else "scene construction failed",
                  configured_control_hz=100, configured_publication_hz=100,
                  achieved_control_hz=100 if successful else 0,
                  achieved_publication_hz=100 if successful else 0,
                  wall_seconds=30 if successful else 0,
                  simulation_seconds=30 if successful else 0, successful_steps=steps,
                  dropped_wall_seconds=0, frame_count=2700 if successful else 0,
                  refresh_hz=90 if successful else 0,
                  box_settled_floor_observed=successful, max_robot_body_displacement_m=.1,
                  global_contact_max=24, gpu_supported=successful, gpu_disjoint_count=0)
    for name in ("physics", "base", "ik", "xr_cpu", "gpu"):
        native[name + "_timing"] = dict(timing)
    return dict(started_utc=started, native=native, traced=False,
                health_before=None, health_after=None)


class PlotExportTests(unittest.TestCase):
    def test_zero_window_and_complete_window_export_with_identical_csv_schema(self):
        # Both chronological orders matter: DictWriter uses the first row's keys.
        for failed_first in (True, False):
            with self.subTest(failed_first=failed_first), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                output = root / "exports"
                for name, success in (("failed", False), ("complete", True)):
                    folder = root / "group" / name
                    folder.mkdir(parents=True)
                    first = (not success) == failed_first
                    data = trial(name, success, "2026-09-05T01:00:00Z" if first else "2026-09-05T02:00:00Z")
                    (folder / "trial.json").write_text(json.dumps(data), encoding="utf-8")
                rows, evidence = plot_rate_sweep.collect(root)
                self.assertEqual(set(rows[0]), set(rows[1]))
                failed = next(r for r in rows if r["run_id"] == "failed")
                complete = next(r for r in rows if r["run_id"] == "complete")
                self.assertFalse(failed["valid"])
                self.assertIsNone(failed["achieved_hz"])
                self.assertIsNone(failed["realtime_ratio"])
                self.assertIsNone(failed["worker_busy_fraction"])
                self.assertFalse(failed["recommended_candidate"])
                self.assertTrue(complete["recommended_candidate"])
                self.assertEqual(complete["achieved_hz"], 200)
                self.assertTrue(complete["gpu_supported"])
                self.assertEqual(len(evidence), 2)
                # Exercise the actual CSV/JSON exporter without requiring plotting
                # dependencies in the standard-library host regression suite.
                with mock.patch("sys.argv", ["plot_rate_sweep", "--root", str(root), "--output", str(output)]), \
                        mock.patch.object(plot_rate_sweep, "plot") as plot, contextlib.redirect_stdout(io.StringIO()):
                    plot_rate_sweep.main()
                plot.assert_called_once()
                with (output / "trials.csv").open(newline="", encoding="utf-8") as file:
                    exported = list(csv.DictReader(file))
                self.assertEqual(len(exported), 2)
                failed_csv = next(r for r in exported if r["run_id"] == "failed")
                self.assertEqual(failed_csv["achieved_hz"], "")
                self.assertEqual(failed_csv["realtime_ratio"], "")
                saved = json.loads((output / "trials.json").read_text(encoding="utf-8"))
                failed_evidence = next(e for e in saved["evidence"] if e["trial"]["native"]["run_id"] == "failed")
                self.assertEqual(failed_evidence["evaluation"]["reason"], "scene construction failed")


if __name__ == "__main__":
    unittest.main()
