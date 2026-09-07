from __future__ import annotations

import math
import re
import tempfile
import unittest
from pathlib import Path

from robot_station.adapters.fafu_kin import (
    URDF_JOINTS,
    URDF_TOOL_XYZ,
    clip_cart_rpy_deg,
    clip_cart_xyz,
    clip_gripper_effort,
    clip_rad,
    forward,
    forward_pose,
    inverse,
    inverse_with_fallback,
    load_gripper_effort,
    matrix_to_rpy,
    rpy_to_matrix,
)

ROOT = Path(__file__).resolve().parents[1]


def _parse_urdf_chain(text: str) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    blocks = re.findall(
        r'<joint name="joint(\d)" type="revolute">(.*?)</joint>',
        text,
        flags=re.S,
    )
    by_id: dict[int, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}
    for idx, body in blocks:
        orig = re.search(r'<origin xyz="([^"]+)"', body)
        axis = re.search(r'<axis xyz="([^"]+)"', body)
        if not orig or not axis:
            continue
        xyz = tuple(float(x) for x in orig.group(1).split())
        ax = tuple(float(x) for x in axis.group(1).split())
        by_id[int(idx)] = (xyz, ax)  # type: ignore[assignment]
    return [by_id[i] for i in range(1, 7)]


def _parse_js_joints(text: str) -> list[tuple[list[float], list[float]]]:
    chunk = text.split("var JOINTS = [", 1)[1].split("];", 1)[0]
    found = re.findall(
        r"xyz:\s*\[([^\]]+)\].*?axis:\s*\[([^\]]+)\]",
        chunk,
        flags=re.S,
    )
    out = []
    for xyz, axis in found:
        out.append(
            (
                [float(x) for x in xyz.split(",")],
                [float(x) for x in axis.split(",")],
            )
        )
    return out


class FafuKinTests(unittest.TestCase):
    def test_home_tool_is_urdf_translation_sum(self) -> None:
        xyz, rpy = forward([0.0] * 6)
        expect = [0.0, 0.0, 0.0]
        for origin, _axis in URDF_JOINTS:
            expect = [expect[i] + origin[i] for i in range(3)]
        expect = [expect[i] + URDF_TOOL_XYZ[i] for i in range(3)]
        self.assertAlmostEqual(xyz[0], expect[0], places=9)
        self.assertAlmostEqual(xyz[1], expect[1], places=9)
        self.assertAlmostEqual(xyz[2], expect[2], places=9)
        self.assertAlmostEqual(xyz[0], 0.246649, places=6)
        self.assertAlmostEqual(xyz[2], 0.168719, places=6)
        self.assertAlmostEqual(rpy[0], 0.0, places=9)
        self.assertAlmostEqual(rpy[1], 0.0, places=9)
        self.assertAlmostEqual(rpy[2], 0.0, places=9)

    def test_table_matches_sdk_urdf(self) -> None:
        from robot_station.adapters.fafu_arm import default_urdf_path, resolve_sdk_root

        try:
            resolve_sdk_root()
        except FileNotFoundError:
            self.skipTest("本机未安装 fafu_arm_sdk")
        urdf = default_urdf_path()
        self.assertIsNotNone(urdf)
        assert urdf is not None
        self.assertTrue(urdf.is_file(), urdf)
        parsed = _parse_urdf_chain(urdf.read_text(encoding="utf-8"))
        self.assertEqual(len(parsed), 6)
        for i, ((xyz, axis), (ref_xyz, ref_axis)) in enumerate(zip(parsed, URDF_JOINTS)):
            for a, b in zip(xyz, ref_xyz):
                self.assertAlmostEqual(a, b, places=9, msg=f"joint{i+1} xyz")
            for a, b in zip(axis, ref_axis):
                self.assertAlmostEqual(a, b, places=9, msg=f"joint{i+1} axis")
        text = urdf.read_text(encoding="utf-8")
        self.assertTrue('xyz="0.165 0 0"' in text or "0.165" in text, "tool/link offset missing")

    def test_table_matches_arm3d_js(self) -> None:
        js = (ROOT / "robot_station" / "web" / "static" / "js" / "arm3d.js").read_text(encoding="utf-8")
        parsed = _parse_js_joints(js)
        self.assertEqual(len(parsed), 6)
        for i, ((xyz, axis), (ref_xyz, ref_axis)) in enumerate(zip(parsed, URDF_JOINTS)):
            for a, b in zip(xyz, ref_xyz):
                self.assertAlmostEqual(a, b, places=6, msg=f"js joint{i+1} xyz")
            for a, b in zip(axis, ref_axis):
                self.assertAlmostEqual(a, b, places=6, msg=f"js joint{i+1} axis")
        self.assertIn("0.165", js)

    def test_roundtrip_mid_pose(self) -> None:
        q = [0.0, math.radians(40.0), math.radians(40.0), 0.1, -0.2, 0.3]
        xyz, rpy = forward(q)
        solved = inverse(xyz, rpy, q)
        self.assertIsNotNone(solved)
        assert solved is not None
        xyz2, rpy2 = forward(solved)
        self.assertAlmostEqual(xyz[0], xyz2[0], places=4)
        self.assertAlmostEqual(xyz[1], xyz2[1], places=4)
        self.assertAlmostEqual(xyz[2], xyz2[2], places=4)
        for a, b in zip(rpy, rpy2):
            self.assertAlmostEqual(a, b, places=3)
        for a, b in zip(q, solved):
            self.assertAlmostEqual(a, b, places=3)

    def test_clip_respects_j1_floor(self) -> None:
        clipped = clip_rad([0.0, math.radians(-20.0), 0.0, 0.0, 0.0, 0.0])
        self.assertGreaterEqual(clipped[1], math.radians(-0.01))

    def test_clip_cart_box(self) -> None:
        xyz = clip_cart_xyz([9.0, -9.0, 9.0])
        self.assertAlmostEqual(xyz[0], 0.75)
        self.assertAlmostEqual(xyz[1], -0.55)
        self.assertAlmostEqual(xyz[2], 0.70)
        rpy = clip_cart_rpy_deg([200.0, -200.0, 0.0])
        self.assertAlmostEqual(rpy[0], 180.0)
        self.assertAlmostEqual(rpy[1], -180.0)

    def test_step_cartesian_plus_x(self) -> None:
        from robot_station.adapters.fafu_kin import step_cartesian, xyz_err

        q = [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0]
        xyz0, _ = forward(q)
        nxt = step_cartesian(q, [0.01, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.assertIsNotNone(nxt)
        assert nxt is not None
        xyz1, _ = forward(nxt)
        self.assertGreater(xyz1[0], xyz0[0] + 0.004)
        self.assertLess(xyz_err(xyz1, [xyz0[0] + 0.01, xyz0[1], xyz0[2]]), 0.008)
        q = [0.0] * 6
        xyz, _rpy = forward(q)
        nxt = inverse_with_fallback(
            [xyz[0] + 0.005, xyz[1], xyz[2]],
            seed_rad=q,
            rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            is_euler=False,
        )
        self.assertIsNotNone(nxt)
        assert nxt is not None
        xyz2, _ = forward(nxt)
        self.assertGreater(xyz2[0], xyz[0] + 0.003)
        q = [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0]
        xyz, rpy = forward(q)
        nxt = inverse([xyz[0] + 0.02, xyz[1], xyz[2]], rpy, q)
        self.assertIsNotNone(nxt)
        assert nxt is not None
        xyz2, _ = forward(nxt)
        self.assertGreater(xyz2[0], xyz[0] + 0.015)
        self.assertLess(abs(xyz2[0] - (xyz[0] + 0.02)), 0.002)

    def test_tracik_optional_matches_station_fk(self) -> None:
        from robot_station.adapters.fafu_tracik import available, inverse_tracik

        if not available():
            self.skipTest("未安装 pytracik（笛卡尔仍用站控 DLS IK）")
        q = [0.0, math.radians(40.0), math.radians(40.0), 0.1, -0.2, 0.3]
        xyz, rot, rpy = forward_pose(q)
        solved = inverse_tracik(xyz, rot, q)
        self.assertIsNotNone(solved)
        assert solved is not None
        xyz2, _rot2, rpy2 = forward_pose(solved)
        self.assertAlmostEqual(xyz[0], xyz2[0], places=3)
        self.assertAlmostEqual(xyz[1], xyz2[1], places=3)
        self.assertAlmostEqual(xyz[2], xyz2[2], places=3)
        for a, b in zip(rpy, rpy2):
            self.assertAlmostEqual(a, b, places=2)
        via = inverse_with_fallback(xyz, seed_rad=q, rotation=rot, is_euler=False)
        self.assertIsNotNone(via)

    def test_rpy_matrix_roundtrip(self) -> None:
        rpy = [0.2, -0.3, 0.4]
        back = matrix_to_rpy(rpy_to_matrix(rpy))
        for a, b in zip(rpy, back):
            self.assertAlmostEqual(a, b, places=9)

    def test_fake_pose_is_rotation_matrix(self) -> None:
        from robot_station.adapters.fafu_sim import FakeFafuController

        fake = FakeFafuController("robot.cfg", auto_enable=False, allow_motion=True)
        pos, rot = fake.get_pose()
        self.assertEqual(len(pos), 3)
        self.assertEqual(len(rot), 3)
        self.assertEqual(len(rot[0]), 3)
        xyz, r_mat, _ = forward_pose(fake.q_rad)
        for i in range(3):
            self.assertAlmostEqual(pos[i], xyz[i], places=9)
            for j in range(3):
                self.assertAlmostEqual(rot[i][j], r_mat[i][j], places=9)

    def test_load_cfg_limits_matches_robot_cfg(self) -> None:
        from robot_station.adapters.fafu_arm import default_cfg_path, resolve_sdk_root
        from robot_station.adapters.fafu_kin import load_cfg_limits

        try:
            cfg = default_cfg_path(resolve_sdk_root())
        except FileNotFoundError:
            self.skipTest("本机未安装 fafu_arm_sdk")
        joints, grip = load_cfg_limits(cfg, 6)
        self.assertGreater(joints[1][1], 170.0)
        self.assertGreater(joints[2][1], 200.0)
        self.assertGreaterEqual(grip[1], grip[0])
        self.assertGreater(grip[1], 70.0)
        effort = load_gripper_effort(cfg)
        self.assertGreaterEqual(effort, 50)
        self.assertLessEqual(effort, 800)

    def test_clip_and_load_gripper_effort(self) -> None:
        self.assertEqual(clip_gripper_effort(None), 300)
        self.assertEqual(clip_gripper_effort(10), 50)
        self.assertEqual(clip_gripper_effort(9999), 800)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "robot.cfg"
            path.write_text("# comment\ngripper_max_torque_raw = 95\n", encoding="utf-8")
            self.assertEqual(load_gripper_effort(path), 95)
            self.assertEqual(load_gripper_effort(None), 300)

    def test_matches_pinocchio_when_installed(self) -> None:
        try:
            import pinocchio as pin  # type: ignore
        except Exception:
            self.skipTest("pinocchio 未安装（真机笛卡尔回退到站控 URDF IK）")
        from robot_station.adapters.fafu_arm import default_urdf_path, resolve_sdk_root

        try:
            resolve_sdk_root()
        except FileNotFoundError:
            self.skipTest("本机未安装 fafu_arm_sdk")
        urdf = default_urdf_path()
        self.assertIsNotNone(urdf)
        assert urdf is not None
        model = pin.buildModelFromUrdf(str(urdf))
        data = model.createData()
        fid = model.getFrameId("tool_link")
        samples = (
            [0.0] * 6,
            [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0],
            [0.3, math.radians(50.0), math.radians(70.0), -0.4, 0.2, -0.15],
        )
        for q in samples:
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            pose = data.oMf[fid]
            xyz, rot, _ = forward_pose(q)
            for i in range(3):
                self.assertAlmostEqual(xyz[i], float(pose.translation[i]), places=9)
            r_pin = pose.rotation
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(rot[i][j], float(r_pin[i, j]), places=9)


if __name__ == "__main__":
    unittest.main()
