from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

from tools.wrs_legacy_preview import wrs_view


class WrsViewTests(unittest.TestCase):
    def tearDown(self) -> None:
        wrs_view.stop()

    def test_start_without_wrs_root(self) -> None:
        old = os.environ.pop("WRS_ROOT", None)
        try:
            err = wrs_view.start("http://127.0.0.1:9/api/info")
        finally:
            if old is not None:
                os.environ["WRS_ROOT"] = old
        self.assertIsNotNone(err)
        self.assertIn("WRS_ROOT", err or "")
        self.assertFalse(wrs_view.status()["running"])

    def test_rewrite_mesh_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            meshes = root / "meshes"
            meshes.mkdir()
            stl = meshes / "base_link.STL"
            stl.write_bytes(b"solid")
            urdf = root / "arm.urdf"
            urdf.write_text(
                """<?xml version="1.0"?>
<robot name="arm">
  <link name="base_link">
    <collision><geometry>
      <mesh filename="package://fafu_robot_description/meshes/base_link.STL"/>
    </geometry></collision>
  </link>
</robot>
""",
                encoding="utf-8",
            )
            rewritten, by_link = wrs_view.rewrite_urdf_meshes(urdf, root / "out.urdf")
            text = rewritten.read_text(encoding="utf-8")
            self.assertNotIn("package://", text)
            self.assertEqual(by_link["base_link"].resolve(), stl.resolve())
            self.assertIn(str(stl.resolve()), text)

    def test_rewrite_missing_mesh(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            urdf = Path(tmp) / "arm.urdf"
            urdf.write_text(
                """<robot name="arm"><link name="base_link">
                <collision><geometry>
                <mesh filename="package://fafu_robot_description/meshes/missing.STL"/>
                </geometry></collision></link></robot>""",
                encoding="utf-8",
            )
            with self.assertRaises(RuntimeError) as ctx:
                wrs_view.rewrite_urdf_meshes(urdf)
            self.assertIn("missing.STL", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
