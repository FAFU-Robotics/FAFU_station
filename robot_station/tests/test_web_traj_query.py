from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class WebTrajQueryTests(unittest.TestCase):
    def test_traj_routes_keep_the_parsed_query(self) -> None:
        # dispatch() already reduced the request to parsed.path; re-parsing it
        # would drop ?name=&to= and make DELETE/PATCH silently act on "".
        server = (ROOT / "robot_station" / "web" / "server.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("urlsplit(path).query", server)
        self.assertEqual(server.count("qs = query"), 2)
