from __future__ import annotations

import asyncio
import contextlib
import json
import queue
import threading
import unittest
import time
from types import SimpleNamespace
from unittest.mock import patch

from robot_station.config import StationConfig
from robot_station.telem import unpack_telem
from robot_station.web.server import _telem
from robot_station.web import server


class TelemetryBackpressureTests(unittest.IsolatedAsyncioTestCase):
    async def test_latest_pose_does_not_discard_an_unsent_full_status(self):
        frames = queue.Queue()
        consumed = {2: threading.Event(), 3: threading.Event()}
        clock = [1.0]

        class Station:
            latest = 0

            def wait_snap(self, last_seq, *_args):
                try:
                    seq = frames.get(timeout=0.02)
                except queue.Empty:
                    return last_seq, {}
                self.latest = seq
                clock[0] = {1:1.0, 2:2.0, 3:2.01}[seq]
                if seq in consumed:
                    consumed[seq].set()
                return seq, {"seq":seq,"arm":{"q_deg":[seq]*6,"feedback_deg":[seq]*6},
                             "program":{"status":"completed" if seq == 2 else "running"}}

            def teleop_hot(self):
                return self.latest == 3  # Frame 3 is deliberately pose-only.

        class Socket:
            def __init__(self):
                self.first = asyncio.Event()
                self.release = asyncio.Event()
                self.latest = asyncio.Event()
                self.sent = []

            async def send(self, body):
                pose = unpack_telem(body) if isinstance(body, bytes) else None
                if pose and pose["seq"] == 1:
                    self.first.set()
                    await self.release.wait()
                self.sent.append(body)
                if pose and pose["seq"] == 3:
                    self.latest.set()

        socket = Socket()
        frames.put(1)
        fake_time = patch.object(server, "time", SimpleNamespace(monotonic=lambda:clock[0], sleep=time.sleep))
        fake_time.start()
        task = asyncio.create_task(_telem(socket, Station(), StationConfig()))
        try:
            await asyncio.wait_for(socket.first.wait(), 2)
            await asyncio.sleep(0.12)  # A fresh full status is now due.
            frames.put(2)
            self.assertTrue(await asyncio.to_thread(consumed[2].wait, 2))
            await asyncio.sleep(0.02)
            frames.put(3)
            self.assertTrue(await asyncio.to_thread(consumed[3].wait, 2))
            await asyncio.sleep(0.02)
            socket.release.set()
            await asyncio.wait_for(socket.latest.wait(), 2)
            poses = [unpack_telem(body)["seq"] for body in socket.sent if isinstance(body, bytes)]
            statuses = [json.loads(body)["d"] for body in socket.sent if isinstance(body, str)]
            self.assertEqual(poses, [1, 3], "pose traffic should still coalesce to the latest frame")
            self.assertEqual(statuses[-1]["seq"], 2)
            self.assertEqual(statuses[-1]["program"]["status"], "completed")
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
            fake_time.stop()

    async def test_joint_follow_still_receives_orientation_and_measured_feedback(self):
        class Station:
            def wait_snap(self, last_seq, *_args):
                if last_seq == 1:
                    threading.Event().wait(0.02)
                return 1, {"seq":1,"arm":{"q_deg":[0]*6,"feedback_deg":[1]*6,
                                           "ee_rpy_deg":[10,20,30]}}

            def teleop_hot(self):
                return True

        class Socket:
            def __init__(self):
                self.hud = asyncio.Event()
                self.status = None

            async def send(self, body):
                if isinstance(body, str):
                    self.status = json.loads(body)["d"]
                    self.hud.set()

        socket = Socket()
        task = asyncio.create_task(_telem(socket, Station(), StationConfig()))
        try:
            await asyncio.wait_for(socket.hud.wait(), 2)
            self.assertEqual(socket.status["arm"]["ee_rpy_deg"], [10,20,30])
            self.assertEqual(socket.status["arm"]["feedback_deg"], [1]*6)
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
