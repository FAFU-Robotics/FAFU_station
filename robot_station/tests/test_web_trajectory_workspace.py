from __future__ import annotations

import json
import math
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
from robot_station.service import Station
from robot_station.traj import TrajectoryWriter, load_traj, make_traj_path
from robot_station.web.server import serve_http
from tests.test_traj import recordings_tmpdir


@unittest.skipUnless(shutil.which("node"), "node not installed")
class TrajectoryStationBrowserTests(unittest.TestCase):
    def test_full_station_ui_against_simulated_robot(self):
        # Bind only the loopback interface; never construct a USB controller.
        with recordings_tmpdir() as tmp:
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            cfg = StationConfig(arm="sim", camera="mock", listen_host="127.0.0.1", http_port=port, open_browser=False)
            station = Station(cfg)
            stop = threading.Event()
            worker = threading.Thread(target=serve_http, args=(station, cfg, stop), daemon=True)
            station.start()
            station.arm.set_powered(True)
            snap = station.arm.poll()
            rows = [{"t":i/100,"pos":[math.radians(q) for q in snap.q_deg],
                     "gpos":math.radians(snap.gripper_deg),"vel":[0]*6} for i in range(201)]
            for i, row in enumerate(rows):
                row["pos"][0] += math.sin(i/40)*0.15
                row["pos"][1] += math.sin(i/60)*0.1
            source = make_traj_path("integration-source")
            writer = TrajectoryWriter(source, teach="soft", mode="Position", n=6, prefix=rows)
            writer.close()
            original = source.read_bytes()
            worker.start()
            base = f"http://127.0.0.1:{port}"
            try:
                deadline = time.monotonic()+5
                while True:
                    try:
                        with urlopen(base+"/api/traj", timeout=1) as response:
                            self.assertEqual(len(json.load(response)["files"]),1)
                        break
                    except OSError:
                        if time.monotonic()>deadline:raise
                        time.sleep(0.05)
                result = subprocess.run([shutil.which("node"), str(Path(__file__).parent/"js"/"trajectories.test.js")],
                                        capture_output=True, text=True, encoding="utf-8", timeout=90,
                                        env={**os.environ,"FAFU_REAL_STATION_URL":base})
                if "SKIP:" in result.stdout:self.skipTest(result.stdout.strip())
                self.assertEqual(result.returncode,0,result.stdout+"\n"+result.stderr)
                self.assertEqual(source.read_bytes(),original)
                _,edited=load_traj(tmp/"integration-edited.jsonl")
                self.assertAlmostEqual(edited[-1]["t"],1.6)
                scoped_header,scoped=load_traj(tmp/"integration-scoped.jsonl")
                self.assertEqual(scoped_header["edit_history"]["operations"][-1]["scope"],"selection")
                self.assertTrue(any(op["type"]=="select" for op in scoped_header["edit_history"]["operations"]))
                self.assertEqual(len(scoped),len(edited))
                for before,after in zip(edited,scoped):
                    if before["t"]<0.3 or before["t"]>0.8:
                        self.assertEqual(before["pos"],after["pos"])
                        self.assertEqual(before["t"],after["t"])
                _,recorded=load_traj(tmp/"integration-recorded.jsonl")
                self.assertGreater(len(recorded),2)
                continued=[x for x in tmp.glob("integration-recorded_*.jsonl")]
                self.assertEqual(len(continued),1)
                _,longer=load_traj(continued[0])
                self.assertGreater(len(longer),len(recorded))
                with urlopen(base+"/api/traj/download?name=integration-edited.jsonl") as response:
                    self.assertEqual(response.read(),(tmp/"integration-edited.jsonl").read_bytes())
                    self.assertIn("attachment",response.headers["Content-Disposition"])
            finally:
                stop.set();worker.join(timeout=5);station.stop()
