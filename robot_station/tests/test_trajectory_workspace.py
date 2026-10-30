from __future__ import annotations

import copy
import math
import time
import unittest

from robot_station.traj import TrajectoryWriter, load_traj, make_traj_path
from robot_station.trajectory_workspace import edit_frames, process_trajectory, validate_frames, velocities
from robot_station.config import StationConfig
from robot_station.motion import MotionCore
from tests.test_traj import recordings_tmpdir


def frames(count=101):
    return [{"t": i/100, "pos": [i/100, 0.7, 0.7, 0, 0, 0],
             "gpos": 0.5, "vel": [0]*6} for i in range(count)]


class TrajectoryEditingTests(unittest.TestCase):
    def test_selection_history_restores_ranges_and_does_not_change_samples(self):
        rows=frames(301)
        original=copy.deepcopy(rows)
        selected=edit_frames(rows,{"type":"select","start":1,"end":2})
        self.assertEqual(selected,original)
        with recordings_tmpdir():
            source=make_traj_path("selection-history")
            writer=TrajectoryWriter(source,teach="soft",mode="Position",n=6,prefix=original);writer.close()
            original_bytes=source.read_bytes()
            operations=[{"type":"select","start":1,"end":2},
                        {"type":"speed","scope":"selection","start":1,"end":2,"factor":2},
                        {"type":"select","start":0.2,"end":0.4}]
            saved=process_trajectory(source.name,operations,save=True)
            self.assertEqual(saved["editor_version"],3)
            reopened=process_trajectory(saved["name"])
            self.assertEqual(reopened["selection"],{"start":0.2,"end":0.4})
            self.assertAlmostEqual(reopened["duration"],2.5)
            previous=process_trajectory(saved["name"],operations[:2])
            self.assertEqual(previous["selection"],{"start":1,"end":1.5})
            only_selection=process_trajectory(saved["name"],operations[:1])
            self.assertEqual(only_selection["selection"],{"start":1,"end":2})
            self.assertEqual([x["pos"] for x in only_selection["frames"]],[x["pos"] for x in original])
            baseline=process_trajectory(saved["name"],[])
            self.assertEqual(baseline["selection"],{"start":0,"end":3})
            self.assertEqual(source.read_bytes(),original_bytes)

    def test_startup_fingerprint_tracks_both_trajectory_modules(self):
        import tempfile
        from pathlib import Path
        from robot_station.runtime import station_code_rev
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            package=root/"robot_station";package.mkdir()
            baseline=station_code_rev(root)
            (package/"trajectory_workspace.py").write_text("EDITOR_VERSION = 3",encoding="utf-8")
            edit_changed=station_code_rev(root)
            self.assertNotEqual(baseline,edit_changed)
            (package/"traj.py").write_text("PLAYBACK_DT = 0.01",encoding="utf-8")
            self.assertNotEqual(edit_changed,station_code_rev(root))

    def test_selected_smoothing_preserves_every_sample_outside_range(self):
        rows=frames(301)
        rows[50]["pos"][0]+=0.4
        rows[150]["pos"][0]+=0.4
        original=copy.deepcopy(rows)
        result=edit_frames(rows,{"type":"smooth","scope":"selection","start":1,"end":2,"window":9})
        self.assertLess(result[150]["pos"][0],1.6)
        for before,after in zip(original,result):
            if before["t"]<=1 or before["t"]>=2:
                self.assertEqual(before["pos"],after["pos"])
                self.assertEqual(before["t"],after["t"])
            self.assertEqual(before["gpos"],after["gpos"])

    def test_selected_speed_ripples_later_time_without_changing_poses(self):
        rows=frames(301)
        result=edit_frames(rows,{"type":"speed","scope":"selection","start":1,"end":2,"factor":2})
        self.assertAlmostEqual(result[-1]["t"],2.5)
        for before,after in zip(rows,result):
            self.assertEqual(before["pos"],after["pos"])
            if before["t"]<1:self.assertEqual(before["t"],after["t"])
            elif before["t"]>2:self.assertAlmostEqual(before["t"]-0.5,after["t"])
        # Saved histories from previous versions explicitly retain global edits.
        self.assertAlmostEqual(edit_frames(frames(301),{"type":"speed","factor":2})[-1]["t"],1.5)

    def test_selected_idle_only_removes_the_selected_pause(self):
        rows=frames(301)
        for row in rows:row["pos"][0]=min(row["t"],1)
        result=edit_frames(rows,{"type":"idle","scope":"selection","start":1.2,"end":1.8,"threshold":0.5,"minimum":0.3})
        self.assertAlmostEqual(result[-1]["t"],2.41)
        prefix=[x for x in result if x["t"]<1.2]
        self.assertEqual([x["pos"] for x in prefix],[x["pos"] for x in rows if x["t"]<1.2])
        # The final stationary second lies outside the selection and is retained.
        self.assertEqual(len([x for x in result if x["t"]>1.41]),100)
        self.assertEqual(result[-1]["pos"],rows[-1]["pos"])

    def test_selected_resampling_keeps_outside_frames_and_exact_boundaries(self):
        rows=frames(301)
        result=edit_frames(rows,{"type":"resample","scope":"selection","start":0.755,"end":1.225,"hz":10})
        for before in rows:
            if before["t"]<0.755 or before["t"]>1.225:
                after=next(x for x in result if abs(x["t"]-before["t"])<1e-9)
                self.assertEqual(before["pos"],after["pos"])
        self.assertAlmostEqual(result[-1]["t"],3)
        self.assertLess(len([x for x in result if 0.755<=x["t"]<=1.225]),10)

    def test_selected_reverse_adds_bridges_and_keeps_outside_poses(self):
        rows=frames(301)
        result=edit_frames(rows,{"type":"reverse","scope":"selection","start":1,"end":2,"transition":1})
        self.assertAlmostEqual(result[-1]["t"],5)
        self.assertEqual([x["pos"] for x in result if x["t"]<1],[x["pos"] for x in rows if x["t"]<1])
        self.assertEqual([x["pos"] for x in result if x["t"]>4],[x["pos"] for x in rows if x["t"]>2])
        self.assertLess(max(abs(b["pos"][0]-a["pos"][0]) for a,b in zip(result,result[1:])),0.02)
        self.assertTrue(all(b["t"]>a["t"] for a,b in zip(result,result[1:])))

    def test_selected_bounds_and_unsupported_merge_scope_are_rejected(self):
        for start,end in ((2,1),(-1,1),(0,4),(1,1),(math.nan,2)):
            with self.assertRaises(ValueError):
                edit_frames(frames(301),{"type":"smooth","scope":"selection","start":start,"end":end})
        with self.assertRaises(ValueError):edit_frames(frames(),{"type":"merge","scope":"selection"})
        with self.assertRaises(ValueError):edit_frames(frames(),{"type":"smooth","scope":"unknown"})

    def test_saved_selection_history_and_result_range_survive_reopen(self):
        with recordings_tmpdir():
            source=make_traj_path("selected")
            writer=TrajectoryWriter(source,teach="soft",mode="Position",n=6,prefix=frames(301));writer.close()
            op={"type":"speed","scope":"selection","start":1,"end":2,"factor":2}
            saved=process_trajectory(source.name,[op],save=True)
            self.assertEqual(saved["selection"],{"start":1,"end":1.5})
            reopened=process_trajectory(saved["name"])
            self.assertEqual(reopened["operations"],[op])
            self.assertAlmostEqual(reopened["duration"],2.5)
            self.assertEqual(reopened["selection"],saved["selection"])
            self.assertAlmostEqual(process_trajectory(saved["name"],[])["duration"],3)

    def test_trim_interpolates_boundaries_and_rebases_time(self):
        result = edit_frames(frames(), {"type":"trim", "start":0.125, "end":0.875})
        self.assertEqual(result[0]["t"], 0)
        self.assertAlmostEqual(result[-1]["t"], 0.75)
        self.assertAlmostEqual(result[0]["pos"][0], 0.125)
        self.assertAlmostEqual(result[-1]["pos"][0], 0.875)

    def test_delete_bridges_gap_and_never_jumps_in_one_tick(self):
        result = edit_frames(frames(), {"type":"delete", "start":0.2, "end":0.8, "transition":1})
        self.assertEqual(result[0]["pos"][0], 0)
        self.assertEqual(result[-1]["pos"][0], 1)
        self.assertGreater(result[-1]["t"], 1)
        self.assertLess(max(abs(b["pos"][0]-a["pos"][0]) for a,b in zip(result,result[1:])),0.02)
        result = edit_frames(frames(), {"type":"delete", "start":0, "end":0.8})
        self.assertEqual(result[0]["t"],0)
        with self.assertRaises(ValueError):
            edit_frames(frames(), {"type":"delete", "start":0, "end":1})

    def test_idle_removal_retains_gripper_motion_and_detects_slow_drift(self):
        stationary = frames()
        for row in stationary:
            row["pos"][0] = 0
        result = edit_frames(stationary, {"type":"idle", "threshold":0.5, "minimum":0.5})
        self.assertLess(result[-1]["t"],0.02)
        moving_gripper = copy.deepcopy(stationary)
        for i,row in enumerate(moving_gripper):
            row["gpos"] = i/100
        result = edit_frames(moving_gripper, {"type":"idle", "threshold":0.5, "minimum":0.5})
        self.assertAlmostEqual(result[-1]["t"],1)
        # Per-tick changes are small but cumulative movement is not stationary.
        result = edit_frames(frames(), {"type":"idle", "threshold":0.5, "minimum":0.5})
        self.assertAlmostEqual(result[-1]["t"],1)

    def test_smoothing_preserves_endpoints_and_gripper(self):
        noisy = frames()
        noisy[50]["pos"][0] += 0.3
        original = copy.deepcopy(noisy)
        result = edit_frames(noisy, {"type":"smooth", "window":9})
        self.assertLess(result[50]["pos"][0],0.6)
        self.assertEqual(result[0]["pos"],original[0]["pos"])
        self.assertEqual(result[-1]["pos"],original[-1]["pos"])
        self.assertEqual([x["gpos"] for x in result],[x["gpos"] for x in original])

    def test_speed_reverse_resample_and_gradients(self):
        result = velocities(edit_frames(frames(), {"type":"speed", "factor":0.5}))
        self.assertEqual(result[-1]["t"],2)
        self.assertAlmostEqual(result[50]["vel"][0],0.5)
        result = edit_frames(frames(), {"type":"reverse"})
        self.assertEqual(result[0]["pos"][0],1)
        self.assertEqual(result[-1]["pos"][0],0)
        result = edit_frames(frames(104), {"type":"resample", "hz":30})
        self.assertAlmostEqual(result[-1]["t"],1.03)
        self.assertEqual(result[-1]["pos"][0],1.03)
        self.assertLess(len(result),40)

    def test_invalid_data_and_parameters_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            bad=frames();bad[3]["pos"][1]=value
            with self.assertRaises(ValueError):validate_frames(bad)
            with self.assertRaises(ValueError):edit_frames(frames(),{"type":"speed","factor":value})
        bad=frames();bad[2]["t"]=0
        with self.assertRaises(ValueError):validate_frames(bad)
        bad=frames();bad[2]["pos"]=[0]
        with self.assertRaises(ValueError):validate_frames(bad)

    def test_legacy_equal_timestamps_keep_last_measurement(self):
        rows=frames()
        rows.insert(2,{**rows[1],"pos":[0.25,0.7,0.7,0,0,0]})
        result=validate_frames(rows)
        self.assertEqual(len(result),101)
        self.assertEqual(result[1]["pos"][0],0.25)

    def test_save_merge_full_resolution_and_source_remains_unchanged(self):
        with recordings_tmpdir():
            source=make_traj_path("original")
            writer=TrajectoryWriter(source,teach="soft",mode="Position",n=6,prefix=frames(4001));writer.close()
            original=source.read_bytes()
            preview=process_trajectory(source.name)
            self.assertLessEqual(len(preview["frames"]),3001)
            self.assertEqual(preview["count"],4001)
            result=process_trajectory(source.name,[{"type":"merge","file":source.name,"transition":1}],save=True,dest="合并")
            _,saved=load_traj(source.parent/result["name"])
            self.assertGreater(len(saved),8000)
            self.assertEqual(source.read_bytes(),original)
            self.assertEqual(saved[-1]["pos"],frames(4001)[-1]["pos"])
            self.assertTrue(all(b["t"]>a["t"] for a,b in zip(saved,saved[1:])))
            again=process_trajectory(source.name,save=True,dest="合并")
            self.assertNotEqual(result["name"],again["name"])

    def test_prefix_writer_appends_after_existing_timeline_and_refuses_overwrite(self):
        with recordings_tmpdir():
            path=make_traj_path("continued")
            writer=TrajectoryWriter(path,teach="soft",mode="Position",n=6,prefix=frames())
            writer.log([1]*6,[0]*6,0.5);writer.close()
            _,samples=load_traj(path)
            self.assertEqual(len(samples),102)
            self.assertGreater(samples[-1]["t"],1)
            with self.assertRaises(FileExistsError):
                TrajectoryWriter(path,teach="soft",mode="Position",n=6)

    def test_saved_history_survives_reopen_rename_and_deleted_sources(self):
        with recordings_tmpdir() as tmp:
            source=make_traj_path("source")
            writer=TrajectoryWriter(source,teach="soft",mode="Position",n=6,prefix=frames());writer.close()
            operations=[{"type":"trim","start":0.2,"end":0.8},{"type":"speed","factor":0.5}]
            saved=process_trajectory(source.name,operations,save=True,dest="edited")
            source.unlink()
            reopened=process_trajectory(saved["name"])
            self.assertEqual(reopened["operations"],operations)
            self.assertAlmostEqual(reopened["duration"],1.2)
            original=process_trajectory(saved["name"],[])
            self.assertAlmostEqual(original["duration"],1)
            undone=process_trajectory(saved["name"],operations[:1],future=operations[1:],save=True,dest="undone")
            reopened=process_trajectory(undone["name"])
            self.assertEqual(reopened["future"],operations[1:])
            redone=process_trajectory(undone["name"],reopened["operations"]+reopened["future"])
            self.assertAlmostEqual(redone["duration"],1.2)
            # Renaming the visible file does not alter its immutable baseline.
            from robot_station.traj import rename_recording
            renamed=rename_recording(saved["name"],"renamed")
            self.assertAlmostEqual(process_trajectory(renamed.name,[])["duration"],1)

    def test_merge_history_freezes_the_other_trajectory_and_export_remains_playable(self):
        with recordings_tmpdir() as tmp:
            source=make_traj_path("source")
            writer=TrajectoryWriter(source,teach="soft",mode="Position",n=6,prefix=frames());writer.close()
            other=make_traj_path("other")
            writer=TrajectoryWriter(other,teach="soft",mode="Position",n=6,prefix=frames());writer.close()
            saved=process_trajectory(source.name,[{"type":"merge","file":other.name,"transition":1}],save=True)
            other.unlink();source.unlink()
            self.assertAlmostEqual(process_trajectory(saved["name"])["duration"],3)
            self.assertAlmostEqual(process_trajectory(saved["name"],[])["duration"],1)
            # A standalone export still works if copied without the archive.
            header,_=load_traj(tmp/saved["name"])
            from robot_station.trajectory_workspace import snapshot_path
            snapshot_path(header["edit_history"]["source"]).unlink()
            flattened=process_trajectory(saved["name"])
            self.assertTrue(flattened["history_warning"])
            self.assertAlmostEqual(flattened["duration"],3)


class TrajectoryMotionTests(unittest.TestCase):
    def test_sim_continue_guards_endpoint_and_preserves_source(self):
        with recordings_tmpdir():
            core=MotionCore(StationConfig(arm="sim"))
            core.arm.start()
            try:
                core.arm.set_powered(True)
                snap=core.arm.poll()
                rows=frames()
                for row in rows:
                    row["pos"]=[math.radians(q) for q in snap.q_deg]
                    row["gpos"]=math.radians(snap.gripper_deg) if snap.gripper_deg is not None else None
                path=make_traj_path("source")
                writer=TrajectoryWriter(path,teach="soft",mode="Position",n=6,prefix=rows);writer.close()
                original=path.read_bytes()
                cmd=lambda action,**kw:core.on_workbench("ui",{"action":action,"request_id":"id",**kw})
                reply=cmd("traj_record",file=path.name,teach="soft")
                self.assertTrue(reply["ok"],reply)
                name=reply["data"]["name"]
                self.assertNotEqual(name,path.name)
                for _ in range(5):core.arm.servo(0.01)
                self.assertFalse(cmd("traj_process",file=path.name)["ok"])
                reply=cmd("traj_record_stop")
                self.assertTrue(reply["ok"],reply)
                _,continued=load_traj(path.parent/name)
                self.assertGreater(len(continued),len(rows))
                self.assertEqual(path.read_bytes(),original)
                self.assertTrue(cmd("traj_process",file=name)["ok"])
                rows[-1]["pos"][0]+=0.2
                far=make_traj_path("far")
                writer=TrajectoryWriter(far,teach="soft",mode="Position",n=6,prefix=rows);writer.close()
                reply=cmd("traj_record",file=far.name,teach="soft")
                self.assertFalse(reply["ok"],reply)
                self.assertIn("3°",reply["error"])
                core.on_estop()
                self.assertFalse(cmd("traj_record",file=path.name,teach="soft")["ok"])
            finally:
                core.arm.stop()


if __name__ == "__main__":
    unittest.main()
