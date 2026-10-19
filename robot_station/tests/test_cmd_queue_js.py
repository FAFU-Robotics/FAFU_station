from __future__ import annotations

import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = shutil.which("node")


@unittest.skipUnless(NODE, "node not installed")
class CmdQueueJsTests(unittest.TestCase):
    def test_queue_semantics(self) -> None:
        script = ROOT / "tests" / "js" / "cmd_queue.test.js"
        self.assertTrue(script.is_file(), script)
        proc = subprocess.run(
            [NODE, str(script)],
            capture_output=True,
            text=True,
            timeout=60,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + "\n" + proc.stderr)
