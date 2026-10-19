from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class WrsRemovedFromProductTests(unittest.TestCase):
    def test_wrs_is_not_in_the_station_product_path(self) -> None:
        html = (ROOT / "robot_station" / "web" / "static" / "index.html").read_text(
            encoding="utf-8"
        )
        js = (ROOT / "robot_station" / "web" / "static" / "js" / "app.js").read_text(
            encoding="utf-8"
        )
        server = (ROOT / "robot_station" / "web" / "server.py").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("WRS仿真", html)
        self.assertNotIn('data-page="wrs"', html)
        self.assertNotIn("btnWrsStart", js)
        self.assertNotIn("wrsAction", js)
        self.assertNotIn("/api/wrs", server)
        self.assertNotIn("wrs_view", server)
        self.assertFalse((ROOT / "robot_station" / "wrs_view.py").exists())
        self.assertFalse((ROOT / "tests" / "test_wrs_view.py").exists())


if __name__ == "__main__":
    unittest.main()
