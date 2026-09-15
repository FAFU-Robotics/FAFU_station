from __future__ import annotations

import os
import shutil
import tempfile
import unittest
from pathlib import Path

from robot_station.adapters.fafu_dyn import LINK_COM, LINK_MASS, gravity_torque
from robot_station.grav_urdf import (
    list_gravity_urdfs,
    load_urdf_dynamics,
    persist_selected_urdf,
    resolve_gravity_urdf,
    selected_urdf_name,
    urdf_dir,
)

ROOT = Path(__file__).resolve().parents[1]
BUNDLED = ROOT / "urdf"


class GravUrdfTests(unittest.TestCase):
    def setUp(self) -> None:
        self._old = os.environ.get("STATION_URDF_DIR")
        self._tmp = tempfile.mkdtemp(prefix="station-urdf-")
        os.environ["STATION_URDF_DIR"] = self._tmp

    def tearDown(self) -> None:
        if self._old is None:
            os.environ.pop("STATION_URDF_DIR", None)
        else:
            os.environ["STATION_URDF_DIR"] = self._old
        shutil.rmtree(self._tmp, ignore_errors=True)

    def _copy(self, name: str) -> Path:
        src = BUNDLED / name
        dest = Path(self._tmp) / name
        shutil.copy2(src, dest)
        return dest

    def test_bundled_folder_has_official_models(self) -> None:
        self.assertTrue((BUNDLED / "fafu_baseV1.urdf").is_file())
        self.assertTrue((BUNDLED / "fafu_baseV1_teach.urdf").is_file())
        self.assertTrue((BUNDLED / "fafu_baseV1_d405.urdf").is_file())

    def test_list_and_select(self) -> None:
        self._copy("fafu_baseV1.urdf")
        self._copy("fafu_baseV1_teach.urdf")
        names = list_gravity_urdfs()
        self.assertEqual(names, ["fafu_baseV1.urdf", "fafu_baseV1_teach.urdf"])
        self.assertEqual(selected_urdf_name(), "fafu_baseV1.urdf")
        persist_selected_urdf("fafu_baseV1_teach.urdf")
        self.assertEqual(selected_urdf_name(), "fafu_baseV1_teach.urdf")
        path = resolve_gravity_urdf("fafu_baseV1_teach.urdf")
        self.assertEqual(path, Path(self._tmp) / "fafu_baseV1_teach.urdf")

    def test_rejects_path_escape(self) -> None:
        self._copy("fafu_baseV1.urdf")
        with self.assertRaises(FileNotFoundError):
            resolve_gravity_urdf("../secrets.urdf")
        with self.assertRaises(FileNotFoundError):
            resolve_gravity_urdf("nope.urdf")

    def test_parse_matches_v1_table(self) -> None:
        dyn = load_urdf_dynamics(BUNDLED / "fafu_baseV1.urdf")
        for i in range(6):
            self.assertAlmostEqual(dyn.masses[i], LINK_MASS[i], places=6)
            for j in range(3):
                self.assertAlmostEqual(dyn.coms[i][j], LINK_COM[i][j], places=6)

    def test_teach_mass_changes_gravity(self) -> None:
        teach = load_urdf_dynamics(BUNDLED / "fafu_baseV1_teach.urdf")
        self.assertGreater(teach.masses[1], LINK_MASS[1])
        self.assertGreater(teach.masses[5], LINK_MASS[5])
        q = [0.0, 0.7, 0.7, 0.0, 0.0, 0.0]
        g0 = gravity_torque(q)
        g1 = gravity_torque(q, masses=teach.masses, coms=teach.coms, joints=teach.joints)
        self.assertGreater(abs(g1[1] - g0[1]) + abs(g1[2] - g0[2]), 0.02)

    def test_urdf_dir_honors_env(self) -> None:
        self.assertEqual(urdf_dir(), Path(self._tmp))

    def test_motion_selects_urdf(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.motion import MotionCore

        self._copy("fafu_baseV1.urdf")
        self._copy("fafu_baseV1_teach.urdf")
        core = MotionCore(StationConfig(arm="mock", open_browser=False))
        self.assertIsNone(core.on_grav_urdf("a", "fafu_baseV1_teach.urdf"))
        snap = core.arm.poll()
        self.assertEqual(snap.urdf, "fafu_baseV1_teach.urdf")
        ack = core.handle({"t": "grav_urdf", "cid": "a", "name": "missing.urdf"})
        self.assertIsNotNone(ack)
        assert ack is not None
        self.assertFalse(ack["ok"])
