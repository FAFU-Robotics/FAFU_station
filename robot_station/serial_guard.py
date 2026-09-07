"""Hardware interlock: never open the real arm serial unless explicitly allowed.

Instantiating ``FafuRobotController`` opens the USB debug board even with
``auto_enable=False``. Default is deny so a first launch on the customer PC
cannot seize the port. Set ``STATION_ALLOW_LIVE_ARM=1`` only when this
program should own the serial link.
"""
from __future__ import annotations

import logging
import os

logger = logging.getLogger("station.serial")

_TRUE = frozenset({"1", "true", "yes", "on"})


def live_serial_allowed() -> bool:
    raw = (os.environ.get("STATION_ALLOW_LIVE_ARM") or "").strip().lower()
    return raw in _TRUE


def apply_live_arm_policy(kind: str) -> str:
    """Map live backends to sim when serial is forbidden. mock/sim unchanged."""
    name = (kind or "mock").strip().lower()
    if name in ("fafu", "auto") and not live_serial_allowed():
        logger.warning(
            "未允许真机串口：拒绝 arm=%s 打开 USB，改用 sim。"
            "真机请设置 STATION_ALLOW_LIVE_ARM=1",
            name,
        )
        return "sim"
    return name
