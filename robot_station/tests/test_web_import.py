from __future__ import annotations

import unittest


class WebImport(unittest.TestCase):
    def test_server_imports(self) -> None:
        from robot_station.web import server

        self.assertTrue(hasattr(server, "serve_http"))
        self.assertTrue(hasattr(server, "serve"))
