from __future__ import annotations

import shutil
import subprocess
import unittest
from pathlib import Path


@unittest.skipUnless(shutil.which("node"), "node not installed")
class WorkbenchJsTests(unittest.TestCase):
    def test_connection_interactions(self) -> None:
        script = Path(__file__).parent / "js" / "connections.test.js"
        result = subprocess.run(
            [shutil.which("node"), str(script)],
            capture_output=True, text=True, encoding="utf-8", timeout=60,
        )
        if "SKIP:" in result.stdout:
            self.skipTest(result.stdout.strip())
        self.assertEqual(result.returncode, 0, result.stdout + "\n" + result.stderr)

    def test_program_interactions(self) -> None:
        script = Path(__file__).parent / "js" / "programs.test.js"
        result = subprocess.run(
            [shutil.which("node"), str(script)],
            capture_output=True, text=True, encoding="utf-8", timeout=90,
        )
        if "SKIP:" in result.stdout:
            self.skipTest(result.stdout.strip())
        self.assertEqual(result.returncode, 0, result.stdout + "\n" + result.stderr)

    def test_trajectory_interactions(self) -> None:
        script = Path(__file__).parent / "js" / "trajectories.test.js"
        result = subprocess.run(
            [shutil.which("node"), str(script)],
            capture_output=True, text=True, encoding="utf-8", timeout=90,
        )
        if "SKIP:" in result.stdout:
            self.skipTest(result.stdout.strip())
        self.assertEqual(result.returncode, 0, result.stdout + "\n" + result.stderr)

    def test_waypoint_interactions(self) -> None:
        script = Path(__file__).parent / "js" / "waypoints.test.js"
        result = subprocess.run(
            [shutil.which("node"), str(script)],
            capture_output=True, text=True, encoding="utf-8", timeout=60,
        )
        if "SKIP:" in result.stdout:
            self.skipTest(result.stdout.strip())
        self.assertEqual(result.returncode, 0, result.stdout + "\n" + result.stderr)
