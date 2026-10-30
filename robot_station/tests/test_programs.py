from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_station.config import StationConfig
from robot_station.motion import MotionCore
from robot_station.programs import ProgramRunner, WorkbenchStore, format_script, parse_script, validate_tree
from robot_station.world import ArmSnap


class ScriptTests(unittest.TestCase):
    def test_tree_script_roundtrip_with_unicode_and_both_branches(self):
        source = '''comment("取放程序")
for _ in range(2):
    movej("取料点", speed=15)
    if gripper_is_open():
        gripper_close(effort=120)
    else:
        gripper_open()
    sleep(0.2)
replay("示教.jsonl", rate=0.8, speed=20)
'''
        first = parse_script(source)
        second = parse_script(format_script(first))
        self.assertEqual(first, second)

    def test_empty_groups_roundtrip(self):
        nodes = validate_tree([{"type": "loop", "count": 2, "children": []},
                               {"type": "if", "children": [], "otherwise": []}])
        self.assertEqual(nodes, parse_script(format_script(nodes)))

    def test_rejects_executable_python_and_nonfinite_parameters(self):
        invalid = ["import os", "__import__('os').system('whoami')", "open('file')",
                   "movej(q)", "movej([0]*6)", "sleep(1e999)", "sleep(-1)",
                   "for _ in range(2):\n    __import__('os')", "while True:\n    pass",
                   "gripper_open(**{})", "movej('P', speed=0)", "if True:\n    sleep(1)"]
        for source in invalid:
            with self.subTest(source=source), self.assertRaises(ValueError):
                parse_script(source)

    def test_rejects_duplicate_ids_and_excessive_nesting(self):
        with self.assertRaises(ValueError):
            validate_tree([{"id": "same", "type": "wait"}, {"id": "same", "type": "wait"}])
        node = {"type": "wait"}
        for _ in range(10):
            node = {"type": "loop", "count": 1, "children": [node]}
        with self.assertRaises(ValueError):
            validate_tree([node])


class StorageTests(unittest.TestCase):
    def test_program_and_waypoints_persist_and_paths_are_confined(self):
        with tempfile.TemporaryDirectory() as raw:
            store = WorkbenchStore(Path(raw))
            points = [{"name": "取料点", "q": [0, 40, 40, 0, 0, 0], "t": 2}]
            store.save_points(points)
            nodes = parse_script('movej("取料点", speed=20)')
            store.save("取放", nodes)
            reopened = WorkbenchStore(Path(raw))
            self.assertEqual(reopened.points(), points)
            self.assertEqual(reopened.load("取放")["nodes"], nodes)
            self.assertEqual(reopened.catalog()["programs"], ["取放"])
            for name in ("../escape", "..\\escape", "/absolute", ""):
                with self.subTest(name=name), self.assertRaises(ValueError):
                    store.save(name, nodes)
            store.delete("取放")
            self.assertEqual(store.catalog()["programs"], [])

    def test_invalid_points_do_not_replace_existing_data(self):
        with tempfile.TemporaryDirectory() as raw:
            store = WorkbenchStore(Path(raw))
            original = [{"name": "P", "q": [0, 40, 40, 0, 0, 0], "t": 2}]
            store.save_points(original)
            for points in ([original[0], original[0]], [{"name": "P", "q": [float("nan")]*6}]):
                with self.assertRaises(ValueError):
                    store.save_points(points)
                self.assertEqual(store.points(), original)


class RunnerTests(unittest.TestCase):
    def test_move_waits_for_feedback_instead_of_commanded_pose(self):
        runner = ProgramRunner()
        runner.start("P", parse_script("movej([10,40,40,0,0,0])\ngripper_close()"), [], None)
        sent = []
        arm = ArmSnap(q_deg=[10,40,40,0,0,0], feedback_deg=[0,40,40,0,0,0])
        for tick in range(30):
            runner.tick(tick/100, arm, lambda n: sent.append(n["type"]))
        self.assertEqual(sent, ["movej"])
        self.assertEqual(runner.completed, 0)
        arm.feedback_deg = [10,40,40,0,0,0]
        for tick in range(30, 50):
            runner.tick(tick/100, arm, lambda n: sent.append(n["type"]))
        self.assertEqual(sent, ["movej", "gripper"])

    def test_loops_and_conditions_are_evaluated_at_execution(self):
        runner = ProgramRunner()
        runner.start("P", parse_script("for _ in range(2):\n    if gripper_is_open():\n        gripper_close()\n    else:\n        gripper_open()"), [], None)
        arm, sent = ArmSnap(), []
        def dispatch(node):
            sent.append(node["open"])
            arm.gripper_open = node["open"]
        for tick in range(150):
            runner.tick(tick/100, arm, dispatch)
        self.assertEqual(sent, [False, True])
        self.assertEqual(runner.status, "completed")

    def test_preflight_checks_all_references_before_any_motion(self):
        runner = ProgramRunner()
        with self.assertRaisesRegex(ValueError, "找不到路点"):
            runner.start("P", parse_script("gripper_open()\nmovej('missing')"), [], None)
        self.assertEqual(runner.status, "idle")
        with self.assertRaisesRegex(ValueError, "限位"):
            runner.start("P", parse_script("movej([0,-10,40,0,0,0])"), [], [[-180,180],[0,180]] + [[-180,180]]*4)
        with self.assertRaisesRegex(ValueError, "10000"):
            runner.start("P", parse_script("for _ in range(1000):\n    for _ in range(1000):\n        sleep(0)"), [], None)

    def test_single_step_and_pause_resume_do_not_skip_actions(self):
        runner = ProgramRunner()
        runner.start("P", parse_script("sleep(0)\ngripper_close()"), [], None, single=True)
        arm, sent = ArmSnap(), []
        for tick in range(30):
            runner.tick(tick/100, arm, lambda n: sent.append(n["type"]))
        self.assertEqual(runner.status, "paused")
        self.assertEqual(sent, ["wait"])
        runner.resume()
        for tick in range(30, 40):
            runner.tick(tick/100, arm, lambda n: sent.append(n["type"]))
        self.assertEqual(sent, ["wait", "gripper"])
        runner.pause()
        self.assertEqual(runner.status, "paused")
        runner.resume()
        runner.tick(0.5, arm, lambda n: sent.append(n["type"]))
        self.assertEqual(sent, ["wait", "gripper", "gripper"])
        runner.stop()
        runner.tick(1.0, arm, lambda n: sent.append("unexpected"))
        self.assertNotIn("unexpected", sent)

    def test_gripper_waits_for_endpoint_or_stationary_contact(self):
        runner = ProgramRunner()
        runner.start("open", parse_script("gripper_open()"), [], None)
        arm = ArmSnap(gripper_open=True, gripper_deg=40, gripper_limits=[8,75], gripper_moving=False)
        for tick in range(50):
            runner.tick(tick/100, arm, lambda n: None)
        self.assertEqual(runner.completed, 0)
        arm.gripper_deg = 75
        for tick in range(50,65):
            runner.tick(tick/100, arm, lambda n: None)
        self.assertEqual(runner.status, "completed")
        runner.start("close", parse_script("gripper_close(effort=300)"), [], None)
        arm.gripper_deg, arm.gripper_tau_raw = 45, 295
        for tick in range(70,120):
            runner.tick(tick/100, arm, lambda n: None)
        self.assertEqual(runner.status, "completed")


class MotionProgramTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store_patch = patch("robot_station.programs.recordings_dir", return_value=Path(self.tmp.name))
        self.store_patch.start()
        self.core = MotionCore(StationConfig(arm="mock"))
        self.core.add_client("operator")
        self.now = time.monotonic()

    def tearDown(self):
        self.store_patch.stop()
        self.tmp.cleanup()

    def command(self, action, **payload):
        return self.core.handle({"t": "workbench", "cid": "operator", "action": action, **payload})

    def advance(self, seconds):
        for _ in range(int(seconds * 100)):
            self.now += 0.01
            self.core._program_touch = self.now
            self.core._step(self.now, 0.01)

    def test_program_completes_motion_and_gripper_sequentially(self):
        nodes = parse_script("movej([10,40,40,0,0,0], speed=20)\ngripper_close()\nsleep(0.1)")
        self.assertTrue(self.command("run", name="demo", nodes=nodes)["ok"])
        self.advance(1.5)
        self.assertEqual(self.core.program.status, "completed", self.core.program.error)
        self.assertEqual(self.core.program.completed, 3)
        self.assertFalse(self.core.arm.poll().gripper_open)
        self.assertAlmostEqual(self.core.arm.poll().q_deg[0], 10)

    def test_manual_control_cannot_steal_program_and_stop_aborts_motion(self):
        self.assertTrue(self.command("run", nodes=parse_script("movej([90,40,40,0,0,0], speed=5)\ngripper_close()"))["ok"])
        self.advance(0.3)
        error = self.core.on_arm_targets("other", [-50,40,40,0,0,0], 20)
        self.assertIn("程序", error)
        self.assertIn("程序", self.core.on_home("other", 20))
        self.assertIn("程序", self.core.on_mode("other", "Gravity"))
        self.assertIn("程序", self.core.on_clear_estop("other"))
        self.assertTrue(self.command("stop")["ok"])
        stopped = self.core.arm.poll().q_deg[0]
        self.advance(1)
        self.assertAlmostEqual(self.core.arm.poll().q_deg[0], stopped)
        self.assertTrue(self.core.arm.poll().gripper_open)

    def test_owner_disconnect_estop_and_timeout_stop_program(self):
        for mode in ("disconnect", "estop", "timeout"):
            with self.subTest(mode=mode):
                self.core.gate.clear()
                self.core.add_client("operator")
                self.assertTrue(self.command("run", nodes=parse_script("sleep(10)\ngripper_close()"))["ok"])
                self.advance(0.1)
                if mode == "disconnect":
                    self.core.drop_client("operator")
                elif mode == "estop":
                    self.core.on_estop()
                else:
                    self.core._program_touch = self.now - 3
                    self.core._step(self.now, 0.01)
                self.assertEqual(self.core.program.status, "error")
                self.assertTrue(self.core.arm.poll().gripper_open)

    def test_capture_uses_feedback_and_request_ids_are_preserved(self):
        with patch.object(self.core.arm, "poll", return_value=ArmSnap(q_deg=[9]*6, feedback_deg=[2]*6)):
            result = self.core.handle({"t":"workbench", "action":"point_capture", "request_id":"capture-1"})
        self.assertEqual(result["request_id"], "capture-1")
        self.assertEqual(result["data"]["q"], [2]*6)

    def test_simulator_holds_reached_pose_after_program_finishes(self):
        self.core = MotionCore(StationConfig(arm="sim"))
        self.core.arm.start()
        self.core.arm.set_powered(True)
        self.core.add_client("operator")
        try:
            reply = self.command("run", nodes=parse_script("movej([15,40,40,0,0,0], speed=20)\nsleep(0.1)"))
            self.assertTrue(reply["ok"], reply)
            self.advance(2)
            self.assertEqual(self.core.program.status, "completed", self.core.program.error)
            self.advance(0.5)
            self.assertAlmostEqual(self.core.arm.poll().q_deg[0], 15, delta=0.8)
        finally:
            self.core.arm.stop()

    def test_sdk_adapter_inherits_current_sdk_instead_of_copying_it(self):
        try:
            from robot_station.adapters.fafu_arm import import_fafu_sdk
            _pm, cls = import_fafu_sdk()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        import fafu_robot_controller as sdk
        self.assertTrue(issubclass(cls, sdk.FafuRobotController))
        self.assertIs(cls.move_j, sdk.FafuRobotController.move_j)

    def test_torque_estimate_uses_configured_sdk_model(self):
        import sys
        from types import SimpleNamespace
        from robot_station.adapters.fafu_arm import gripper_torque_scale
        robot = SimpleNamespace(_cfg=SimpleNamespace(motor_types={7: "M4438_30"}))
        with patch.dict(sys.modules, {"fafu_motor": SimpleNamespace(TORQUE_COEFF={"M4438_30": 0.64})}):
            self.assertAlmostEqual(gripper_torque_scale(robot, 7), 0.0064)
            self.assertIsNone(gripper_torque_scale(robot, 8))

    def test_webfront_program_rpc_and_state_cross_real_motion_tcp(self):
        import socket
        import threading
        from robot_station.motion import serve_motion
        from robot_station.service import WebFront

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        port = server.getsockname()[1]
        server.listen(4)
        stop = threading.Event()
        worker = threading.Thread(target=serve_motion,
                                  args=(self.core, "127.0.0.1", port, stop, server),
                                  kwargs={"already_listening": True}, daemon=True)
        front = WebFront(StationConfig(arm="mock", camera="mock", motion_port=port))
        self.core.start()
        worker.start()
        try:
            front.motion.connect(timeout_s=2)
            front.add_client("tcp-operator")
            parsed = front.on_workbench("tcp-operator", {
                "action": "parse", "script": "sleep(0.05)\ngripper_close()", "request_id": "wire-1"})
            self.assertTrue(parsed["ok"], parsed)
            self.assertEqual(parsed["request_id"], "wire-1")
            session = self.core.snapshot()["connection"]["session"]
            started = front.on_command("tcp-operator", {
                "t":"workbench", "action": "run", "name": "TCP", "nodes": parsed["data"]["nodes"], "session":session})
            self.assertTrue(started["ok"], started)
            deadline = time.monotonic() + 3
            state = {}
            while time.monotonic() < deadline:
                front.on_command("tcp-operator", {"t":"touch", "session":session})
                state = front.snapshot()
                if state.get("program", {}).get("status") == "completed":
                    break
                time.sleep(0.02)
            self.assertEqual(state.get("program", {}).get("status"), "completed", state)
            self.assertFalse(self.core.arm.poll().gripper_open)
        finally:
            front.motion.close()
            stop.set()
            self.core.stop()
            server.close()
            worker.join(timeout=2)
