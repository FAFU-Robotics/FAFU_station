from __future__ import annotations

import math
import re
import unittest

from robot_station.adapters.fafu_dyn import (
    LINK_COM,
    LINK_MASS,
    compensation_torque,
    friction_torque,
    gravity_torque,
)


def _parse_link_inertial(text: str) -> list[tuple[float, tuple[float, float, float]]]:
    blocks = re.findall(
        r'<link name="link(\d)">\s*<inertial>\s*<origin xyz="([^"]+)"[^/]*/>\s*<mass value="([^"]+)"',
        text,
        flags=re.S,
    )
    by_id: dict[int, tuple[float, tuple[float, float, float]]] = {}
    for idx, xyz, mass in blocks:
        com = tuple(float(x) for x in xyz.split())
        by_id[int(idx)] = (float(mass), com)  # type: ignore[assignment]
    return [by_id[i] for i in range(1, 7)]


class FafuDynTests(unittest.TestCase):
    def test_yaw_joint_has_no_gravity(self) -> None:
        samples = (
            [0.0] * 6,
            [0.4, math.radians(40.0), math.radians(50.0), -0.2, 0.3, -0.1],
            [1.0, math.radians(90.0), math.radians(20.0), 0.5, -0.4, 0.2],
        )
        for q in samples:
            g = gravity_torque(q)
            self.assertAlmostEqual(g[0], 0.0, places=9)
            self.assertEqual(len(g), 6)

    def test_shoulder_holds_against_gravity_at_ready_pose(self) -> None:
        q = [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0]
        g = gravity_torque(q)
        self.assertGreater(abs(g[1]) + abs(g[2]), 1.0)
        self.assertGreater(abs(g[1]), abs(g[5]))

    def test_friction_deadband_drops_coulomb(self) -> None:
        still = friction_torque([0.0, 0.001, 0.0, 0.0, 0.0, 0.0])
        moving = friction_torque([0.0, 0.2, 0.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(still[1], 0.06 * 0.001, places=9)
        self.assertGreater(moving[1], 0.15)

    def test_compensation_clips(self) -> None:
        tau = compensation_torque([0.0] * 6, [0.0] * 6, friction=True, tau_limit=[0.01] * 6)
        for x in tau:
            self.assertLessEqual(abs(x), 0.01 + 1e-12)

    def test_table_matches_sdk_urdf_when_present(self) -> None:
        from robot_station.adapters.fafu_arm import default_urdf_path, resolve_sdk_root

        try:
            resolve_sdk_root()
        except FileNotFoundError:
            self.skipTest("本机未安装 fafu_arm_sdk")
        urdf = default_urdf_path()
        if urdf is None or not urdf.is_file():
            self.skipTest("SDK URDF 不存在")
        parsed = _parse_link_inertial(urdf.read_text(encoding="utf-8"))
        for i, (mass, com) in enumerate(parsed):
            self.assertAlmostEqual(mass, LINK_MASS[i], places=6)
            for j in range(3):
                self.assertAlmostEqual(com[j], LINK_COM[i][j], places=6)

    def test_matches_pinocchio_when_installed(self) -> None:
        try:
            import pinocchio as pin  # type: ignore
        except Exception:
            self.skipTest("pinocchio 未安装")
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
        gvec = model.gravity.linear.copy()
        model.gravity.linear = [0.0, 0.0, -9.81]
        samples = (
            [0.0] * 6,
            [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0],
            [0.3, math.radians(50.0), math.radians(70.0), -0.4, 0.2, -0.15],
        )
        try:
            for q in samples:
                g_pin = pin.computeGeneralizedGravity(model, data, q)
                g = gravity_torque(q)
                for i in range(6):
                    self.assertAlmostEqual(g[i], float(g_pin[i]), places=6)
        finally:
            model.gravity.linear = gvec
