"""Safety latch. ESTOP wins over motion; clear is explicit."""

from __future__ import annotations

from enum import IntEnum
import time


class Safety(IntEnum):
    IDLE = 0
    OPERATING = 1
    ESTOP_LATCHED = 2
    WATCHDOG_TRIP = 3


class SafetyGate:
    def __init__(self, watchdog_s: float) -> None:
        self.watchdog_s = float(watchdog_s)
        self.state = Safety.IDLE
        self.last_cmd_mono = time.monotonic()
        self.reason = ""

    def note_command(self) -> None:
        self.last_cmd_mono = time.monotonic()
        if self.state in (Safety.IDLE, Safety.WATCHDOG_TRIP, Safety.OPERATING):
            self.state = Safety.OPERATING

    def estop(self, reason: str = "operator") -> None:
        self.state = Safety.ESTOP_LATCHED
        self.reason = reason

    def clear(self) -> bool:
        if self.state != Safety.ESTOP_LATCHED:
            return True
        self.state = Safety.IDLE
        self.reason = ""
        self.last_cmd_mono = time.monotonic()
        return True

    def motion_allowed(self) -> bool:
        return self.state != Safety.ESTOP_LATCHED

    def tick_watchdog(self, now: float, moving: bool) -> bool:
        """Return True if the arm must hold this tick."""
        if self.state == Safety.ESTOP_LATCHED:
            return True
        silent = (now - self.last_cmd_mono) > self.watchdog_s
        if silent:
            if moving or self.state == Safety.OPERATING:
                self.state = Safety.WATCHDOG_TRIP
            return True
        return False
