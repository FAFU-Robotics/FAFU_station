from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock
from urllib.request import urlopen

from robot_station.config import StationConfig
from robot_station.service import Station
from robot_station.web.server import _on_text, serve_http
from tests.test_traj import recordings_tmpdir


class WaypointCommandTests(unittest.IsolatedAsyncioTestCase):
    async def test_session_routed_commands_preserve_the_request_id(self):
        for error in (None, "连接会话已变化"):
            with self.subTest(error=error):
                station = SimpleNamespace(on_command=Mock(return_value={
                    "t": "ack", "op": "path", "ok": error is None, "error": error,
                }))
                ws = SimpleNamespace(send=AsyncMock())
                command = {"t":"path", "request_id":"wp-18", "session":"test-session", "q":[[1,40,40,0,0,0]], "speed":20}
                await _on_text(station, "operator", json.dumps(command), ws)
                station.on_command.assert_called_once_with("operator", command)
                reply = json.loads(ws.send.call_args.args[0])
                self.assertEqual(reply["request_id"], "wp-18")
                self.assertEqual(reply["ok"], error is None)

    async def test_motion_acceptance_and_rejection_echo_the_request_id(self):
        for error in (None, "请先切回 Position 再跑路点"):
            with self.subTest(error=error):
                station = SimpleNamespace(on_path=Mock(return_value=error))
                ws = SimpleNamespace(send=AsyncMock())
                await _on_text(station, "operator", json.dumps({
                    "t": "path", "request_id": "wp-17", "q": [[1, 40, 40, 0, 0, 0]], "speed": 20,
                }), ws)
                station.on_path.assert_called_once_with("operator", [[1., 40., 40., 0., 0., 0.]], 20., None)
                reply = json.loads(ws.send.call_args.args[0])
                self.assertEqual(reply["request_id"], "wp-17")
                self.assertEqual(reply["ok"], error is None)
                self.assertEqual(reply["error"], error)


@unittest.skipUnless(shutil.which("node"), "node not installed")
class WaypointSimulationBrowserTests(unittest.TestCase):
    def test_full_ui_confirms_actual_simulated_arrival(self):
        self.run_browser_scenarios()

    def test_manual_home_resend_and_target_replacement(self):
        self.run_browser_scenarios({"FAFU_MANUAL_ONLY": "1"})

    def run_browser_scenarios(self, extra_env=None):
        with recordings_tmpdir():
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            cfg = StationConfig(arm="sim", camera="mock", listen_host="127.0.0.1", http_port=port, open_browser=False)
            station, stop = Station(cfg), threading.Event()
            worker = threading.Thread(target=serve_http, args=(station, cfg, stop), daemon=True)
            station.start()
            self.assertIsNone(station.on_power("test", True))
            worker.start()
            base = f"http://127.0.0.1:{port}"
            try:
                deadline = time.monotonic() + 5
                while True:
                    try:
                        with urlopen(base + "/api/workspace", timeout=1):
                            break
                    except OSError:
                        if time.monotonic() > deadline:
                            raise
                        time.sleep(0.05)
                result = subprocess.run(
                    [shutil.which("node"), str(Path(__file__).parent / "js" / "waypoints.test.js")],
                    capture_output=True, text=True, encoding="utf-8", timeout=60,
                    env={**os.environ, "FAFU_REAL_STATION_URL": base, **(extra_env or {})},
                )
                if "SKIP:" in result.stdout:
                    self.skipTest(result.stdout.strip())
                self.assertEqual(result.returncode, 0, result.stdout + "\n" + result.stderr)
            finally:
                stop.set()
                worker.join(timeout=5)
                station.stop()
