import os
from pathlib import Path
import subprocess
import sys
import unittest

from run_experiments import default_cases, generate_report

ROOT = Path(__file__).resolve().parents[1]


class ExperimentTests(unittest.TestCase):
    def test_identical_reports_with_same_cases_and_seeds(self):
        cases = default_cases(seeds=(2, 4, 8))
        self.assertEqual(generate_report(cases), generate_report(cases))
        self.assertEqual(generate_report(seeds=(2, 4, 8)), generate_report(seeds=(2, 4, 8)))
        self.assertEqual(generate_report(cases), generate_report(seeds=(2, 4, 8)))
        self.assertNotIn("Timing", generate_report())

    def test_report_fields_and_aggregate(self):
        report = generate_report(seeds=())
        self.assertIn("open | yes | ok | 7 | 7 | yes |", report)
        self.assertIn("detour | yes | ok | 10 | 10 | yes |", report)
        self.assertIn("disconnected | no | no_path | - | - | yes |", report)
        self.assertIn("same_cell | yes | ok | 0 | 0 | yes | 0", report)
        self.assertIn("Planning success rate: 3/6 (50.0%)", report)
        self.assertIn("Planning success rate: 0/0 (0.0%)", generate_report([]))

    def run_script(self, *args, hash_seed="1"):
        env = os.environ.copy()
        env["PYTHONHASHSEED"] = hash_seed
        completed = subprocess.run(
            [sys.executable, str(ROOT / "run_experiments.py")] + list(args),
            cwd=str(ROOT / "tests"), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, check=True,
        )
        self.assertEqual(completed.stderr, b"")
        return completed.stdout

    def test_script_byte_identical_across_processes(self):
        first = self.run_script(hash_seed="1")
        second = self.run_script(hash_seed="99")
        self.assertEqual(first, second)
        self.assertEqual(first.decode().replace("\r\n", "\n"), generate_report())

    def test_timing_is_separate(self):
        default = self.run_script()
        timed = self.run_script("--timing")
        self.assertTrue(timed.startswith(default))
        extra = timed[len(default):].decode()
        self.assertIn("Timing (non-deterministic)", extra)
        self.assertRegex(extra, r"validation: \d+\.\d{6} s")


if __name__ == "__main__":
    unittest.main()
