from __future__ import annotations

import json
import socket
import threading
import time
import unittest
from pathlib import Path

from robot_station.bridge import MotionClient
from robot_station.config import ROOT


class TcpRttTests(unittest.TestCase):
    def test_localhost_tcp_roundtrip_under_20ms(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        port = srv.getsockname()[1]
        stop = threading.Event()

        def serve() -> None:
            conn, _addr = srv.accept()
            try:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                buf = b""
                while not stop.is_set():
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        conn.sendall(line + b"\n")
            except OSError:
                pass
            finally:
                conn.close()

        th = threading.Thread(target=serve, daemon=True)
        th.start()
        cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            cli.connect(("127.0.0.1", port))
            cli.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            samples: list[float] = []
            payload = b'{"t":"ping"}\n'
            for _ in range(40):
                t0 = time.perf_counter()
                cli.sendall(payload)
                got = b""
                while b"\n" not in got:
                    got += cli.recv(4096)
                samples.append((time.perf_counter() - t0) * 1000.0)
            median = sorted(samples)[len(samples) // 2]
            self.assertLess(median, 5.0, samples[:8])
            self.assertLess(max(samples), 20.0, samples)
        finally:
            stop.set()
            cli.close()
            srv.close()
            th.join(timeout=1.0)

    def test_station_motion_link_is_already_tcp_nodelay(self) -> None:
        bridge = (ROOT / "robot_station" / "bridge.py").read_text(encoding="utf-8")
        motion = (ROOT / "robot_station" / "motion.py").read_text(encoding="utf-8")
        self.assertIn("TCP_NODELAY", bridge)
        self.assertIn("TCP_NODELAY", motion)
        self.assertIn("127.0.0.1", motion)

    def test_sdk_arm_transport_is_usb_serial_not_tcp(self) -> None:
        spec = (ROOT / "docs" / "SPEC.md").read_text(encoding="utf-8")
        self.assertIn("USB 串口", spec)
        self.assertIn("没有 TCP/以太网控臂", spec)
        arm = (ROOT / "robot_station" / "adapters" / "fafu_arm.py").read_text(encoding="utf-8")
        self.assertIn("kwargs[\"port\"]", arm)
        self.assertNotRegex(arm, r"host\s*=\s*[\"'][0-9]+\.[0-9]+")
        sdk_hits = list(Path(ROOT).parent.glob("fafu_arm_sdk*/fafu_robot_python/robot.cfg"))
        sdk_hits += list((ROOT / "vendor").glob("fafu_arm_sdk*/fafu_robot_python/robot.cfg"))
        for cfg in sdk_hits:
            text = cfg.read_text(encoding="utf-8", errors="ignore")
            self.assertIn("baudrate", text)
            self.assertNotRegex(text.lower(), r"^\s*transport\s*=\s*tcp", msg=str(cfg))

    def test_stream_commands_coalesce_to_latest(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        port = srv.getsockname()[1]
        stop = threading.Event()
        received: list[dict] = []

        def serve() -> None:
            conn, _addr = srv.accept()
            try:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                buf = b""
                while not stop.is_set():
                    try:
                        chunk = conn.recv(4096)
                    except OSError:
                        break
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        received.append(json.loads(line.decode("utf-8")))
            except OSError:
                pass
            finally:
                conn.close()

        th = threading.Thread(target=serve, daemon=True)
        th.start()
        cli = MotionClient("127.0.0.1", port)
        try:
            cli.connect(timeout_s=2.0)
            for i in range(40):
                cli.send({"t": "arm", "stream": True, "cid": "c", "q": [float(i)] * 6, "speed": 40})
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline and not received:
                time.sleep(0.01)
            time.sleep(0.05)
            self.assertGreater(len(received), 0, "coalesced stream never reached TCP")
            self.assertLess(len(received), 40, received)
            self.assertEqual(received[-1]["q"][0], 39.0)
        finally:
            stop.set()
            cli.close()
            srv.close()
            th.join(timeout=1.0)


class StreamCoalesceTests(unittest.TestCase):
    def test_cart_deltas_are_summed(self) -> None:
        from robot_station.bridge import coalesce_stream

        a = {"t": "cart", "dxyz": [0.008, 0.0, 0.0], "drpy": [1.0, 0.0, 0.0]}
        b = {"t": "cart", "dxyz": [0.008, 0.0, 0.0], "drpy": [1.0, 0.0, 0.0]}
        got = coalesce_stream(a, b)
        self.assertAlmostEqual(got["dxyz"][0], 0.016)
        self.assertAlmostEqual(got["drpy"][0], 2.0)

    def test_cart_sum_is_capped(self) -> None:
        from robot_station.bridge import coalesce_stream

        a = {"t": "cart", "dxyz": [0.02, 0.0, 0.0], "drpy": [8.0, 0.0, 0.0]}
        b = {"t": "cart", "dxyz": [0.02, 0.0, 0.0], "drpy": [8.0, 0.0, 0.0]}
        got = coalesce_stream(a, b)
        self.assertAlmostEqual(got["dxyz"][0], 0.025)
        self.assertAlmostEqual(got["drpy"][0], 10.0)

    def test_arm_stream_stays_latest_wins(self) -> None:
        from robot_station.bridge import coalesce_stream

        got = coalesce_stream({"t": "arm", "q": [1.0]}, {"t": "arm", "q": [9.0]})
        self.assertEqual(got["q"], [9.0])


if __name__ == "__main__":
    unittest.main()
