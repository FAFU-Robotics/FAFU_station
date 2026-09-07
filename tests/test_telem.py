from __future__ import annotations

import math
import struct
import unittest

from robot_station.config import ROOT
from robot_station.telem import (
    MAGIC,
    TELEM_SIZE,
    overlay_telem,
    pack_telem,
    take_motion_frames,
    unpack_telem,
)


class TelemCodecTests(unittest.TestCase):
    def test_roundtrip_pose_and_echo(self) -> None:
        src = {
            "seq": 42,
            "echo_t0": 123.25,
            "cmd_hz": 99.5,
            "safety": "ESTOP_LATCHED",
            "arm": {
                "online": True,
                "enabled": False,
                "moving": True,
                "gripper_open": False,
                "ik_err": "nope",
                "q_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                "target_deg": [1.5, 2.5, 3.5, 4.5, 5.5, 6.5],
                "gripper_deg": 40.0,
                "ee_m": [0.11, 0.22, 0.33],
            },
        }
        blob = pack_telem(src)
        self.assertEqual(len(blob), TELEM_SIZE)
        self.assertEqual(TELEM_SIZE, 88)
        got = unpack_telem(blob)
        assert got is not None
        self.assertEqual(got["seq"], 42)
        self.assertAlmostEqual(got["echo_t0"], 123.25, places=5)
        self.assertEqual(got["safety"], "ESTOP_LATCHED")
        self.assertEqual(got["arm"]["q_deg"][5], 6.0)
        self.assertTrue(got["arm"]["moving"])
        self.assertFalse(got["arm"]["enabled"])
        self.assertEqual(got["arm"]["ik_err"], "IK")
        self.assertAlmostEqual(got["arm"]["ee_m"][1], 0.22, places=5)

    def test_js_field_offsets_match_struct(self) -> None:
        src = {
            "seq": 7,
            "echo_t0": 1.5,
            "cmd_hz": 100.0,
            "safety": "OPERATING",
            "arm": {
                "online": True,
                "q_deg": [9.0, 0, 0, 0, 0, 0],
                "target_deg": [8.0, 0, 0, 0, 0, 0],
                "gripper_deg": 33.0,
                "ee_m": [0.4, 0.5, 0.6],
            },
        }
        blob = pack_telem(src)
        self.assertEqual(struct.unpack_from("<I", blob, 0)[0], MAGIC)
        self.assertEqual(struct.unpack_from("<I", blob, 4)[0], 7)
        self.assertEqual(blob[10], 1)
        self.assertAlmostEqual(struct.unpack_from("<d", blob, 12)[0], 1.5, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", blob, 20)[0], 9.0, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", blob, 44)[0], 8.0, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", blob, 68)[0], 33.0, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", blob, 72)[0], 0.4, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", blob, 84)[0], 100.0, places=5)
        js = (ROOT / "robot_station" / "web" / "static" / "js" / "app.js").read_text(encoding="utf-8")
        self.assertIn("getFloat64(12, true)", js)
        self.assertIn("20 + 4 * i", js)
        self.assertIn("44 + 4 * i", js)
        self.assertIn("getFloat32(68, true)", js)
        self.assertIn("72 + 4 * i", js)
        self.assertIn("getFloat32(84, true)", js)

    def test_zero_gripper_deg_roundtrip(self) -> None:
        blob = pack_telem(
            {
                "seq": 4,
                "arm": {"gripper_deg": 0.0, "gripper_open": False, "q_deg": [0] * 6},
            }
        )
        got = unpack_telem(blob)
        assert got is not None
        self.assertAlmostEqual(got["arm"]["gripper_deg"], 0.0, places=5)
        self.assertFalse(got["arm"]["gripper_open"])
        blob = pack_telem({"seq": 1, "arm": {}})
        got = unpack_telem(blob)
        assert got is not None
        self.assertIsNone(got["echo_t0"])
        self.assertFalse(math.isnan(got["arm"]["q_deg"][0]))

    def test_rejects_wrong_size(self) -> None:
        self.assertIsNone(unpack_telem(b"short"))

    def test_overlay_keeps_hud_fields(self) -> None:
        dst = {"arm": {"backend": "sim", "ctrl_mode": "Position", "q_deg": [0] * 6}}
        pose = unpack_telem(pack_telem({"seq": 3, "arm": {"q_deg": [1, 2, 3, 4, 5, 6], "online": True}}))
        assert pose is not None
        got = overlay_telem(dst, pose)
        self.assertEqual(got["arm"]["backend"], "sim")
        self.assertEqual(got["arm"]["q_deg"][0], 1.0)
        self.assertEqual(got["seq"], 3)

    def test_take_motion_frames_mixed(self) -> None:
        pose = pack_telem({"seq": 9, "arm": {"q_deg": [1] * 6}})
        line = b'{"t":"ready","arm":"sim"}\n'
        rest, events = take_motion_frames(line + pose + pose[:20])
        self.assertEqual(len(events), 2)
        self.assertEqual(events[0][0], "json")
        self.assertEqual(events[0][1]["t"], "ready")
        self.assertEqual(events[1][0], "telem")
        self.assertEqual(events[1][1]["seq"], 9)
        self.assertEqual(rest, pose[:20])


if __name__ == "__main__":
    unittest.main()
