from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robot_station.config import StationConfig
from robot_station.diagnostics import connection_diagnostics


class ConnectionDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.board = SimpleNamespace(port="COM7", description="USB board", vid="0483", pid="5740")
        self.other = SimpleNamespace(port="COM3", description="Bluetooth", vid="", pid="")
        self.motor = Mock()
        self.motor.find_likely_debug_boards.return_value = [self.board]
        self.motor.list_serial_ports.return_value = [self.other, self.board]
        self.controller = Mock()
        self.loader = patch("robot_station.diagnostics.import_fafu_sdk", return_value=(self.motor, self.controller))
        self.loader.start()
        self.addCleanup(self.loader.stop)

    def test_enumerates_candidates_without_opening_or_constructing_controller(self):
        report = connection_diagnostics(StationConfig(), {})
        ports = report["serial"]["ports"]
        self.assertEqual([p["port"] for p in ports], ["COM7", "COM3"])
        self.assertTrue(ports[0]["candidate"])
        self.assertFalse(ports[1]["candidate"])
        self.controller.assert_not_called()
        self.assertEqual([call[0] for call in self.motor.method_calls],
                         ["find_likely_debug_boards", "list_serial_ports"])

    def test_empty_enum_does_not_claim_connected_device(self):
        self.motor.list_serial_ports.return_value = []
        self.motor.find_likely_debug_boards.return_value = []
        report = connection_diagnostics(StationConfig(), {"arm": {"online": False, "serial_port": "COM7"}})
        self.assertEqual(report["serial"]["ports"], [])
        self.assertEqual(report["serial"]["active_port"], "")

    def test_sdk_error_is_reported_alongside_service_details(self):
        with patch("robot_station.diagnostics.import_fafu_sdk", side_effect=RuntimeError("extension missing")):
            report = connection_diagnostics(StationConfig(http_port=19400), {}, in_process=True)
        self.assertTrue(report["ok"])
        self.assertFalse(report["serial"]["sdk_loaded"])
        self.assertIn("extension missing", report["serial"]["error"])
        self.assertEqual(report["http"]["port"], 19400)
        self.assertEqual(report["motion"]["mode"], "in_process")

    def test_tcp_connection_is_distinct_from_same_process_simulation(self):
        cfg = StationConfig(motion_port=19470)
        offline = connection_diagnostics(cfg, {})
        self.assertFalse(offline["motion"]["connected"])
        self.assertEqual(offline["motion"]["port"], 19470)
        online = connection_diagnostics(cfg, {}, motion_connected=True)
        self.assertTrue(online["motion"]["connected"])
        self.assertEqual(online["motion"]["mode"], "tcp")

    def test_active_port_comes_from_feedback_not_configured_auto(self):
        report = connection_diagnostics(StationConfig(), {"arm": {"online": True, "serial_port": "COM7"}})
        self.assertEqual(report["serial"]["configured_port"], "")
        self.assertEqual(report["serial"]["active_port"], "COM7")
