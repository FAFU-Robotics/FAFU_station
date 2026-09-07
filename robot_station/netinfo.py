"""IPv4 listing when the host binds to all interfaces.

Default delivery is localhost. These helpers are only for log lines when
listen_host is 0.0.0.0.
"""

from __future__ import annotations

import os
import re
import socket
import subprocess
from typing import Iterable

_ADDR_LINE = re.compile(
    r"^\d+:\s+(\S+)\s+inet\s+(\d{1,3}(?:\.\d{1,3}){3})(?:/(\d{1,2}))?"
)
_SKIP_IFACE = re.compile(
    r"^(lo|docker\d*|br-.*|veth.*|virbr.*|cni.*|flannel.*|tun\d*|tailscale\d*)$",
    re.I,
)


def is_skipped_iface(name: str) -> bool:
    iface = (name or "").split("@", 1)[0].strip()
    return bool(_SKIP_IFACE.match(iface))


def parse_ip_addr(text: str) -> list[tuple[str, str, int | None]]:
    """Parse ``ip -4 -o addr show`` into (iface, ip, prefixlen)."""
    rows: list[tuple[str, str, int | None]] = []
    for raw in (text or "").splitlines():
        m = _ADDR_LINE.search(raw.strip())
        if not m:
            continue
        iface = m.group(1).split("@", 1)[0]
        ip = m.group(2)
        prefix = int(m.group(3)) if m.group(3) else None
        if ip.startswith("127.") or ip.startswith("169.254."):
            continue
        rows.append((iface, ip, prefix))
    return rows


def _ip_o_addr() -> str:
    if os.name == "nt":
        return ""
    try:
        return subprocess.check_output(["ip", "-4", "-o", "addr", "show"], text=True, timeout=1)
    except Exception:
        return ""


def _socket_addrs() -> list[tuple[str, str, int | None]]:
    rows: list[tuple[str, str, int | None]] = []
    seen: set[str] = set()
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET, socket.SOCK_STREAM):
            ip = info[4][0]
            if ip.startswith(("127.", "169.254.")) or ip in seen:
                continue
            seen.add(ip)
            rows.append(("", ip, None))
    except OSError:
        pass
    return rows


def lan_addrs(
    parsed: Iterable[tuple[str, str, int | None]] | None = None,
) -> list[tuple[str, str, int | None]]:
    if parsed is None:
        text = _ip_o_addr()
        parsed = parse_ip_addr(text) if text.strip() else _socket_addrs()
    return [(n, ip, pfx) for n, ip, pfx in parsed if ip and not is_skipped_iface(n)]


def list_lan_ips(parsed: Iterable[tuple[str, str, int | None]] | None = None) -> list[str]:
    """Non-loopback, non-docker IPv4."""
    found: list[str] = []
    seen: set[str] = set()
    for _name, ip, _pfx in lan_addrs(parsed):
        if ip in seen:
            continue
        seen.add(ip)
        found.append(ip)
    return found


def guess_lan_ip(parsed: Iterable[tuple[str, str, int | None]] | None = None) -> str:
    ips = list_lan_ips(parsed)
    return ips[0] if ips else "127.0.0.1"
