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
from urllib.request import urlopen

from robot_station.config import StationConfig
from robot_station.programs import WorkbenchStore
from robot_station.service import Station
from robot_station.web.server import serve_http
from tests.test_traj import recordings_tmpdir


@unittest.skipUnless(shutil.which("node"), "node not installed")
class ProgramStationBrowserTests(unittest.TestCase):
    def test_program_library_and_tree_against_simulated_station(self):
        with recordings_tmpdir():
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            store = WorkbenchStore()
            store.save_points([{"name": "取料点", "q": [15, 40, 40, 0, 0, 0]},
                               {"name": "放置点", "q": [-20, 30, 50, 0, 0, 0]}])
            store.save("取放示例", [{"type": "movej", "point": "取料点", "speed": 20}])
            store.save("返回", [{"type": "movej", "point": "放置点", "speed": 20}])
            cfg = StationConfig(arm="sim", camera="mock", listen_host="127.0.0.1", http_port=port, open_browser=False)
            station = Station(cfg)
            stop = threading.Event()
            worker = threading.Thread(target=serve_http, args=(station, cfg, stop), daemon=True)
            station.start()
            station.arm.set_powered(True)
            worker.start()
            base = f"http://127.0.0.1:{port}"
            try:
                deadline = time.monotonic() + 5
                while True:
                    try:
                        with urlopen(base + "/api/workspace", timeout=1) as response:
                            self.assertEqual(len(json.load(response)["programs"]), 2)
                        break
                    except OSError:
                        if time.monotonic() > deadline:
                            raise
                        time.sleep(0.05)
                result = subprocess.run(
                    [shutil.which("node"), str(Path(__file__).parent / "js" / "programs.test.js")],
                    capture_output=True, text=True, encoding="utf-8", timeout=90,
                    env={**os.environ, "FAFU_REAL_STATION_URL": base},
                )
                if "SKIP:" in result.stdout:
                    self.skipTest(result.stdout.strip())
                self.assertEqual(result.returncode, 0, result.stdout + "\n" + result.stderr)
                nodes = store.load("集成程序")["nodes"]
                self.assertEqual([node["type"] for node in nodes], ["loop", "gripper"])
                self.assertEqual([node["type"] for node in nodes[0]["children"]], ["movej"])
                self.assertEqual(len(store.load("取放示例")["nodes"]), 1)
            finally:
                stop.set()
                worker.join(timeout=5)
                station.stop()
