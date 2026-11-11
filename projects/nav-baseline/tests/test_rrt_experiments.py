import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ReportTests(unittest.TestCase):
    def run_script(self, *args, hash_seed="1"):
        env = os.environ.copy()
        env["PYTHONHASHSEED"] = hash_seed
        result = subprocess.run(
            [sys.executable, str(ROOT / "run_rrt_experiments.py")] + list(args),
            cwd=str(ROOT / "tests"), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, check=True,
        )
        self.assertEqual(result.stderr, b"")
        return result.stdout

    def test_byte_identical_reports_thresholds_and_opt_in_timing(self):
        first = self.run_script(hash_seed="1")
        self.assertEqual(first, self.run_script(hash_seed="99"))
        report = first.decode()
        self.assertNotIn("Timing", report)
        self.assertNotIn("elapsed", report)
        self.assertIn("seeds 0-19", report)
        self.assertIn("max_iterations=2000, step=0.5, goal_bias=0.1", report)
        self.assertIn("median successful length", report)
        self.assertIn("grid reference only", report)
        rows = {parts[0]: parts for parts in (line.split(" | ") for line in report.splitlines()) if len(parts) == 7}
        for name, grid_length in (("open", "7"), ("detour", "10")):
            row = rows[name]
            self.assertGreaterEqual(int(row[1].split("/")[0]), 18)
            self.assertGreater(float(row[2]), 0)
            self.assertEqual(row[4:], [grid_length, ">=18/20", "PASS"])
        self.assertEqual(rows["disconnected"][1:], [
            "0/20", "-", "budget_exhausted=20", "-", "0/20; budget_exhausted=20", "PASS",
        ])
        timed = self.run_script("--timing")
        self.assertTrue(timed.startswith(first))
        extra = timed[len(first):].decode()
        self.assertIn("Timing (non-deterministic)", extra)
        self.assertRegex(extra, r"validation: \d+\.\d{6} s")


if __name__ == "__main__":
    unittest.main()
