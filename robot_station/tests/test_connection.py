from __future__ import annotations

import os
import threading
import time
import unittest
from unittest.mock import Mock, patch

from robot_station.adapters.arm import build_arm
from robot_station.adapters.fafu_arm import FafuArm, resolve_live_serial_port
from robot_station.adapters.fafu_sim import FakeFafuController
from robot_station.config import StationConfig
from robot_station.connection import serial_endpoint
from robot_station.motion import MotionCore


class ConnectionTests(unittest.TestCase):
    def setUp(self):
        self.guard=patch.dict(os.environ,{"STATION_ALLOW_LIVE_ARM":"0"})
        self.guard.start()
        self.cores=[]

    def tearDown(self):
        for core in self.cores:
            core.stop()
        self.guard.stop()

    def core(self,kind="sim",**kwargs):
        core=MotionCore(StationConfig(arm=kind,open_browser=False,control_hz=50,**kwargs))
        self.cores.append(core)
        return core

    def command(self,core,**packet):
        return core.handle({"cid":"operator","session":core.snapshot()["connection"]["session"],**packet})

    @staticmethod
    def fake_build(name,*args,**kwargs):
        if name!="fafu":
            return build_arm(name,*args,**kwargs)
        arm=FafuArm(controller_cls=FakeFafuController,required=True,
            allow_motion=kwargs.get("allow_motion",True),port=kwargs.get("port",""))
        arm.backend="fafu"
        return arm

    def test_serial_start_waits_for_explicit_connection(self):
        core=self.core("fafu",arm_port="COM9",arm_allow_motion=True)
        with patch.object(core.arm,"start") as start:
            core.start()
            self.assertFalse(start.called)
        snap=core.snapshot()
        self.assertEqual(snap["connection"]["kind"],"serial")
        self.assertEqual(snap["connection"]["port"],"COM9")
        self.assertEqual(snap["connection"]["status"],"disconnected")
        self.assertFalse(snap["arm"]["online"])

    def test_explicit_startup_failure_does_not_adopt_simulation(self):
        core=self.core("fafu",arm_port="COM7",arm_connect_on_start=True)
        with patch.object(core.arm,"start",side_effect=RuntimeError("COM7 busy")):
            core.start()
        self.assertEqual(core.arm_kind(),"fafu")
        self.assertEqual(core.snapshot()["connection"]["status"],"error")
        self.assertIn("COM7 busy",core.snapshot()["connection"]["error"])
        before=core.snapshot()["seq"];time.sleep(.06)
        self.assertGreater(core.snapshot()["seq"],before)

    def test_startup_connect_never_enables(self):
        core=self.core("fafu",arm_allow_motion=True,arm_connect_on_start=True)
        core.arm=self.fake_build("fafu",allow_motion=True,port="COM5")
        core.start()
        fake=core.arm._robot
        self.assertNotIn("enable",fake.calls)
        self.assertTrue(core.snapshot()["connection"]["enable_required"])
        self.assertIn("使能",core.on_arm_targets("operator",[5,40,40,0,0,0],20))

    def test_simulation_is_a_disconnectable_connection_with_separate_enable(self):
        core=self.core();core.start()
        self.assertFalse(core.arm.poll().enabled)
        self.assertIn("使能",core.on_home("operator",10))
        self.assertIsNone(core.on_power("operator",True))
        self.assertIsNone(core.on_home("operator",10))
        self.assertTrue(self.command(core,t="link",action="disconnect")["ok"])
        time.sleep(.04)
        self.assertFalse(core.snapshot()["arm"]["online"])
        self.assertIn("未连接",core.on_home("operator",10))
        self.assertTrue(self.command(core,t="link",action="connect",kind="simulation")["ok"])
        self.assertTrue(core.snapshot()["connection"]["enable_required"])
        self.assertFalse(core.arm.poll().enabled)

    def test_connected_target_cannot_be_replaced_or_probed(self):
        core=self.core();core.start()
        before=core.snapshot()["connection"]["session"]
        with patch("robot_station.motion.build_arm") as factory:
            result=self.command(core,t="link",action="connect",kind="serial",port="COM4")
        self.assertFalse(result["ok"])
        self.assertIn("先断开",result["error"])
        self.assertFalse(factory.called)
        self.assertEqual(core.snapshot()["connection"]["session"],before)

    def test_selected_port_is_passed_exactly_and_not_auto_enabled(self):
        core=self.core("fafu",arm_allow_motion=True);core.start()
        for port in ("COM9","COM17"):
            with patch("robot_station.motion.build_arm",side_effect=self.fake_build) as factory, patch("robot_station.serial_guard.live_serial_allowed",return_value=True):
                result=self.command(core,t="link",action="connect",kind="serial",port=port)
            self.assertTrue(result["ok"],result)
            self.assertEqual(factory.call_args.kwargs["port"],port)
            self.assertEqual(core.snapshot()["connection"]["port"],port)
            self.assertNotIn("enable",core.arm._robot.calls)
            self.assertTrue(self.command(core,t="link",action="disconnect")["ok"])

    def test_failed_serial_connection_keeps_selected_target_offline(self):
        core=self.core();core.start()
        self.assertTrue(self.command(core,t="link",action="disconnect")["ok"])
        arm=self.fake_build("fafu",allow_motion=True,port="COM11")
        arm.start=Mock(side_effect=RuntimeError("COM11 unavailable"))
        with patch("robot_station.motion.build_arm",return_value=arm), patch("robot_station.serial_guard.live_serial_allowed",return_value=True):
            result=self.command(core,t="link",action="connect",kind="serial",port="COM11")
        self.assertFalse(result["ok"])
        self.assertEqual(core.arm_kind(),"fafu")
        self.assertEqual(core.snapshot()["connection"]["port"],"COM11")
        self.assertEqual(core.snapshot()["connection"]["status"],"error")
        self.assertFalse(core.snapshot()["arm"]["online"])

    def test_old_commands_and_other_window_requests_are_fenced(self):
        core=self.core();core.start()
        old=core.snapshot()["connection"]["session"]
        self.command(core,t="link",action="disconnect")
        self.command(core,t="link",action="connect",kind="simulation")
        for packet in [{"t":"power","on":True},{"t":"arm","q":[30]*6,"stream":True},
                       {"t":"link","action":"disconnect"},
                       {"t":"workbench","action":"run","request_id":"old-task"}]:
            result=core.handle({"cid":"other-window","session":old,**packet})
            self.assertFalse(result["ok"],result)
            self.assertIn("会话",result["error"])
            if packet["t"]=="workbench":self.assertEqual(result["request_id"],"old-task")
        self.assertNotIn("enable",core.arm._robot.calls)
        self.assertTrue(self.command(core,t="power",on=True)["ok"])

    def test_adapter_construction_failure_cannot_leave_simulation_as_serial_target(self):
        core=self.core();core.start()
        self.command(core,t="link",action="disconnect")
        with patch("robot_station.motion.build_arm",side_effect=RuntimeError("bad SDK configuration")), patch("robot_station.serial_guard.live_serial_allowed",return_value=True):
            result=self.command(core,t="link",action="connect",kind="serial",port="COM33")
        self.assertFalse(result["ok"])
        self.assertEqual(core.arm_kind(),"fafu")
        self.assertEqual(core.snapshot()["connection"]["kind"],"serial")
        self.assertEqual(core.snapshot()["connection"]["port"],"COM33")
        self.assertEqual(core.snapshot()["connection"]["status"],"error")
        self.assertTrue(self.command(core,t="link",action="connect",kind="simulation")["ok"])

    def test_emergency_stop_needs_no_session(self):
        core=self.core();core.start()
        core.handle({"t":"e","session":"obsolete"})
        self.assertEqual(core.snapshot()["safety"],"ESTOP_LATCHED")

    def test_loss_of_last_control_window_invalidates_buffered_motion(self):
        core=self.core();core.start();core.add_client("operator")
        old=core.snapshot()["connection"]["session"]
        core.drop_client("operator")
        self.assertNotEqual(core.snapshot()["connection"]["session"],old)
        self.assertEqual(core.snapshot()["connection"]["status"],"connected")
        self.assertFalse(core.handle({"t":"power","on":True,"session":old})["ok"])
        self.assertFalse(core.arm.poll().enabled)

    def test_failed_enable_is_not_success_just_because_motors_are_online(self):
        core=self.core();core.start()
        core.arm._ensure_enabled=Mock(return_value="使能失败：模拟驱动拒绝使能")
        result=self.command(core,t="power",on=True)
        self.assertFalse(result["ok"])
        self.assertTrue(core.snapshot()["connection"]["enable_required"])
        self.assertTrue(core.arm._connection_enable_pending)
        self.assertFalse(core.arm.poll().enabled)

    def test_new_connection_preserves_existing_motor_modes_until_confirmed(self):
        class AlreadyPowered(FakeFafuController):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                self.is_enabled=True  # Represents a controller already holding the arm.
            def close_connection(self,**kwargs):
                self.close_policy=kwargs
                super().close_connection(**kwargs)
        core=self.core("fafu",arm_allow_motion=True,arm_connect_on_start=True)
        core.arm=FafuArm(controller_cls=AlreadyPowered,allow_motion=True)
        core.arm.backend="fafu"
        core.start()
        robot=core.arm._robot
        self.assertTrue(core.arm.poll().enabled)
        self.assertTrue(core.snapshot()["connection"]["enable_required"])
        self.assertIn("使能",core.on_home("operator",10))
        core.arm.hold();core.arm.resume_motion();core.arm.servo(.02)
        self.assertIsNone(core.on_link("operator","disconnect"))
        self.assertEqual(robot.close_policy["joint_release"],"preserve")
        for method in ("enable","disable","servo_start","servo_j"):
            self.assertNotIn(method,robot.calls)

    def test_preserve_close_only_stops_polling_and_closes_transport(self):
        from robot_station.adapters.fafu_arm import import_fafu_sdk
        _motor,controller=import_fafu_sdk()
        robot=controller.__new__(controller)
        robot._servo_active=False
        robot._ht=Mock()
        robot._ht.is_polling.return_value=True
        robot._ht.is_async_rx.return_value=True
        robot._set_state=Mock()
        robot.close_connection(joint_release="preserve")
        robot._ht.stop_state_polling.assert_called_once()
        robot._ht.disable_async_rx.assert_called_once()
        robot._ht.close.assert_called_once()
        for method in ("stop","brake","set_motor_mode"):
            getattr(robot._ht,method).assert_not_called()

    def test_unplug_invalidates_commands_without_switching_backend(self):
        core=self.core();core.start()
        old=core.snapshot()["connection"]["session"]
        core.program.status="paused"
        with core._arm_lock:
            core.arm.stop()
            core._step(time.monotonic(),.02)
        self.assertEqual(core.arm_kind(),"sim")
        self.assertEqual(core.snapshot()["connection"]["status"],"error")
        self.assertNotEqual(core.snapshot()["connection"]["session"],old)
        self.assertFalse(core.program.active)

    def test_usb_enumeration_never_connects(self):
        core=self.core(arm_watch=True)
        with patch.object(core,"on_arm_src") as switch:
            core._watch_arm(True);core._watch_arm(False)
        self.assertFalse(switch.called)

    def test_tick_keeps_publishing_offline_while_connection_is_busy(self):
        core=self.core();core.start()
        self.command(core,t="link",action="disconnect")
        arm=build_arm("sim",6,40,True)
        start=arm.start;entered=threading.Event();release=threading.Event()
        def slow_start():
            entered.set();release.wait(2);start()
        arm.start=slow_start
        with patch("robot_station.motion.build_arm",return_value=arm):
            worker=threading.Thread(target=lambda:core.on_link("operator","connect","simulation"))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                before=core.snapshot()["seq"];time.sleep(.06)
                self.assertGreater(core.snapshot()["seq"],before)
                self.assertFalse(core.snapshot()["arm"]["online"])
                self.assertEqual(core.snapshot()["connection"]["status"],"connecting")
            finally:
                release.set();worker.join(3)

    def test_explicit_ports_never_fall_back_and_multiple_boards_are_ambiguous(self):
        self.assertEqual(resolve_live_serial_port("COM9",["COM3","COM7"]),"COM9")
        self.assertEqual(resolve_live_serial_port("/dev/ttyUSB8",["/dev/ttyUSB1"]),"/dev/ttyUSB8")
        with self.assertRaisesRegex(RuntimeError,"多个"):
            resolve_live_serial_port("auto",["COM3","COM7"])
        for bad in ("", "auto", "COM0", "COM7\nCOM8", None):
            with self.assertRaises(ValueError):serial_endpoint(bad)


if __name__=="__main__":
    unittest.main()
