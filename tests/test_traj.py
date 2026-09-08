from __future__ import annotations

import json
import os
import tempfile
import time
import unittest
from contextlib import contextmanager
from pathlib import Path

from robot_station.traj import (
    TrajectoryWriter,
    clip_replay_rate,
    delete_recording,
    load_traj,
    make_traj_path,
    prepare_playback,
    recordings_dir,
    resolve_traj_path,
    sample_at,
    sanitize_traj_name,
    list_recordings,
)


@contextmanager
def recordings_tmpdir():
    with tempfile.TemporaryDirectory() as tmp:
        old = os.environ.get("STATION_RECORDINGS_DIR")
        os.environ["STATION_RECORDINGS_DIR"] = tmp
        try:
            yield Path(tmp)
        finally:
            if old is None:
                os.environ.pop("STATION_RECORDINGS_DIR", None)
            else:
                os.environ["STATION_RECORDINGS_DIR"] = old


class TrajFileTests(unittest.TestCase):
    def test_jsonl_roundtrip_skips_header(self) -> None:
        with recordings_tmpdir() as tmp:
            path = make_traj_path("demo")
            writer = TrajectoryWriter(path, teach="soft", mode="Position", n=6)
            writer.log([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 6, 1.2, 0.0)
            time.sleep(0.02)
            writer.log([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], [0.1] * 6, 1.2, 0.0)
            writer.close()
            header, frames = load_traj(path)
            self.assertEqual(header["kind"], "traj")
            self.assertEqual(header["teach"], "soft")
            self.assertEqual(len(frames), 2)
            self.assertAlmostEqual(frames[0]["pos"][0], 0.1)
            self.assertIn("gpos", frames[0])
            listed = list_recordings(tmp)
            self.assertEqual(listed[0]["name"], path.name)
            self.assertEqual(listed[0]["frames"], 2)

    def test_load_sdk_style_without_header(self) -> None:
        with recordings_tmpdir() as tmp:
            path = tmp / "legacy.jsonl"
            with path.open("w", encoding="utf-8") as fh:
                fh.write(json.dumps({"t": 0.0, "pos": [0.0] * 6, "vel": [0.0] * 6}) + "\n")
                fh.write(json.dumps({"t": 0.1, "pos": [0.2] * 6, "vel": [0.0] * 6}) + "\n")
            header, frames = load_traj(path)
            self.assertIsNone(header)
            self.assertEqual(len(frames), 2)

    def test_prepare_playback_resamples_and_smooths(self) -> None:
        samples = []
        for i in range(11):
            t = i * 0.1
            samples.append({"t": t, "pos": [t, 0.0, 0.0, 0.0, 0.0, 0.0], "vel": [9.9] * 6, "gpos": 1.0})
        out = prepare_playback(samples, playback_dt=0.01, smooth_window=7)
        self.assertGreaterEqual(len(out), 90)
        self.assertLessEqual(len(out), 120)
        self.assertAlmostEqual(out[0]["t"], 0.0, places=6)
        self.assertAlmostEqual(out[-1]["pos"][0], 1.0, places=1)
        # Recorded vel is junk; playback vel comes from smoothed position.
        self.assertLess(abs(out[50]["vel"][0]), 2.0)

    def test_sample_at_interpolates(self) -> None:
        frames = [
            {"t": 0.0, "pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "vel": [0.0] * 6},
            {"t": 1.0, "pos": [2.0, 0.0, 0.0, 0.0, 0.0, 0.0], "vel": [0.0] * 6},
        ]
        mid = sample_at(frames, 0.5)
        self.assertAlmostEqual(mid["pos"][0], 1.0)

    def test_sanitize_and_rate(self) -> None:
        self.assertEqual(sanitize_traj_name("ab/../x.jsonl"), "x.jsonl")
        self.assertEqual(sanitize_traj_name("ok-1"), "ok-1.jsonl")
        self.assertEqual(clip_replay_rate(2.0), 1.0)
        self.assertEqual(clip_replay_rate(0.2), 0.5)
        self.assertEqual(clip_replay_rate(0.75), 0.75)

    def test_delete_recording_stays_in_dir(self) -> None:
        with recordings_tmpdir() as tmp:
            path = make_traj_path("gone")
            TrajectoryWriter(path, teach="drag", mode="Gravity", n=6).close()
            self.assertTrue(path.is_file())
            gone = delete_recording(path.name)
            self.assertEqual(gone.name, path.name)
            self.assertFalse(path.is_file())
            with self.assertRaises(FileNotFoundError):
                delete_recording(str(tmp / ".." / "outside.jsonl"))

    def test_resolve_rejects_outside_dir(self) -> None:
        with recordings_tmpdir():
            with self.assertRaises(FileNotFoundError):
                resolve_traj_path("missing.jsonl")
            self.assertTrue(str(recordings_dir()).endswith("recordings") or recordings_dir().is_dir())


if __name__ == "__main__":
    unittest.main()
