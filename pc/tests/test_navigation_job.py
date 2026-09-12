import json

from pc_trajectory.demo.map_model import MapDocument
from pc_trajectory.demo.map_planner import NavigationPlanner
from pc_trajectory.demo.navigation_job import NavigationJob
from pc_trajectory.demo.navigation_simulator import SimulationState, SimulatedNavigationRunner
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Point2D


def test_job_report_and_replay_are_reviewable(tmp_path):
    document = MapDocument.new()
    plan = NavigationPlanner().plan_click_to_go(document, Pose(0.0, 0.0, 0.0), Point2D(80.0, 30.0))
    job = NavigationJob.create(plan, runner=SimulatedNavigationRunner(plan.start_pose))
    job.start()
    snapshot = job.run_to_completion(dt_s=0.02)
    assert snapshot.state is SimulationState.FINISHED

    report_path = job.write_report(tmp_path / "report.json")
    replay_path = job.write_replay(tmp_path / "replay.jsonl")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    replay_lines = replay_path.read_text(encoding="utf-8").splitlines()
    assert report["simulation"]["state"] == "FINISHED"
    assert report["simulation"]["trace_points"] == len(replay_lines)
    assert json.loads(replay_lines[-1])["x_mm"] == 80.0
