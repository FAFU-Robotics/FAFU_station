from __future__ import annotations

import unittest

from robot_station.netinfo import guess_lan_ip, list_lan_ips, parse_ip_addr

SAMPLE_IP_ADDR = """
1: lo    inet 127.0.0.1/8 scope host lo
3: wlP1p1s0    inet 172.18.101.12/21 brd 172.18.103.255 scope global dynamic noprefixroute wlP1p1s0
5: enP8p1s0    inet 192.168.1.102/24 brd 192.168.1.255 scope global noprefixroute enP8p1s0
9: docker0    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
"""


class NetinfoTests(unittest.TestCase):
    def test_skips_loopback_and_docker(self):
        parsed = parse_ip_addr(SAMPLE_IP_ADDR)
        ips = list_lan_ips(parsed)
        self.assertIn("172.18.101.12", ips)
        self.assertIn("192.168.1.102", ips)
        self.assertNotIn("172.17.0.1", ips)
        self.assertNotIn("127.0.0.1", ips)
        self.assertEqual(guess_lan_ip(parsed), "172.18.101.12")

    def test_live_does_not_pick_docker(self):
        self.assertNotEqual(guess_lan_ip(), "172.17.0.1")

    def test_desktop_local_defaults(self):
        import sys
        from pathlib import Path

        root = Path(__file__).resolve().parents[1]
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        import station_desktop as sd

        self.assertEqual(sd.DEFAULT_PORT, 9400)
        self.assertEqual(sd.DEFAULT_PATH, "/")
        self.assertEqual(sd._make_url("127.0.0.1", 9400, "/"), "http://127.0.0.1:9400/")


if __name__ == "__main__":
    unittest.main()
