#!/usr/bin/env python3
"""本机只看总览：不另起一份站控。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_station.config import load_config
from robot_station.lock import port_open


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="打开站控总览（只看）")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)
    cfg = load_config()
    if not port_open(cfg.http_port, "127.0.0.1"):
        print("站控没在跑。请先：")
        print(f"  python3 {ROOT / 'run_station.py'}")
        return 1
    url = f"http://127.0.0.1:{cfg.http_port}/#/home"
    print(url)
    print("这是监视页。不要再开一份站控。")
    if args.no_browser:
        return 0
    try:
        from robot_station.web.server import _open_browser

        _open_browser(url, wait=True)
    except Exception as exc:
        print(f"没能打开浏览器：{exc}。请手动访问上面的地址。")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
