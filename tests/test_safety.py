from __future__ import annotations

import unittest

from robot_station.safety import Safety, SafetyGate


class SafetyTests(unittest.TestCase):
    def test_estop_blocks_until_clear(self) -> None:
        gate = SafetyGate(0.2)
        gate.note_command()
        self.assertTrue(gate.motion_allowed())
        gate.estop("operator")
        self.assertEqual(gate.state, Safety.ESTOP_LATCHED)
        self.assertFalse(gate.motion_allowed())
        gate.note_command()
        self.assertFalse(gate.motion_allowed())
        gate.clear()
        self.assertTrue(gate.motion_allowed())
        self.assertEqual(gate.state, Safety.IDLE)


if __name__ == "__main__":
    unittest.main()
