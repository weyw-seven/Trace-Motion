import math

import pytest

from pc_trajectory.demo.map_model import MapDocument, MapPath
from pc_trajectory.demo.map_planner import NavigationPlanner, PenMode
from pc_trajectory.demo.navigation_simulator import (
    SimulationError,
    SimulationState,
    SimulatedNavigationRunner,
)
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Point2D


def document() -> MapDocument:
    return MapDocument(
        name="sim",
        width_mm=800.0,
        height_mm=500.0,
        paths=(MapPath((Point2D(0.0, 0.0), Point2D(100.0, 0.0), Point2D(180.0, 40.0)), path_id="p"),),
    )


def test_click_plan_runs_to_exact_endpoint_without_teleporting():
    plan = NavigationPlanner().plan_click_to_go(document(), Pose(0.0, 0.0, 0.0), Point2D(120.0, 50.0))
    runner = SimulatedNavigationRunner(plan.start_pose, sample_step_mm=2.0)
    runner.start(plan)
    assert runner.state is SimulationState.RUNNING
    first = runner.tick(0.1)
    assert first.pose.x_mm > 0.0
    assert first.pose.x_mm < 120.0
    final = runner.run_to_completion(dt_s=0.01)
    assert final.state is SimulationState.FINISHED
    assert math.isclose(final.pose.x_mm, 120.0, abs_tol=1.0e-9)
    assert math.isclose(final.pose.y_mm, 50.0, abs_tol=1.0e-9)
    assert final.pen_state.value == "UP"
    assert len(runner.trace) > 2


def test_draw_plan_updates_pen_state_and_trace():
    plan = NavigationPlanner().plan_selected_path(
        document(), Pose(0.0, 0.0, 0.0), "p", pen_mode=PenMode.DRAW
    )
    runner = SimulatedNavigationRunner(plan.start_pose)
    runner.start(plan)
    runner.tick(0.01)
    assert runner.snapshot().pen_state.value == "DOWN"
    final = runner.run_to_completion(dt_s=0.02)
    assert final.state is SimulationState.FINISHED
    assert runner.trace[-1].x_mm == pytest.approx(plan.end_point.x_mm, abs=1.0e-6)
    assert runner.trace[-1].y_mm == pytest.approx(plan.end_point.y_mm, abs=1.0e-6)


def test_stop_and_estop_are_deterministic_and_require_clear():
    plan = NavigationPlanner().plan_click_to_go(document(), Pose(0.0, 0.0, 0.0), Point2D(200.0, 0.0))
    runner = SimulatedNavigationRunner(plan.start_pose)
    runner.start(plan)
    runner.tick(0.1)
    before = runner.pose
    stopped = runner.stop()
    assert stopped.state is SimulationState.STOPPED
    runner.tick(1.0)
    assert runner.pose == before

    runner.reset(plan.start_pose)
    runner.start(plan)
    runner.tick(0.1)
    estopped = runner.emergency_stop()
    assert estopped.state is SimulationState.ESTOPPED
    with pytest.raises(SimulationError, match="emergency stop"):
        runner.start(plan)
    assert runner.clear_emergency_stop().state is SimulationState.IDLE


def test_runner_refuses_a_plan_that_would_teleport_from_current_pose():
    plan = NavigationPlanner().plan_click_to_go(document(), Pose(10.0, 0.0, 0.0), Point2D(100.0, 0.0))
    runner = SimulatedNavigationRunner(Pose(0.0, 0.0, 0.0))
    with pytest.raises(SimulationError, match="does not match plan start"):
        runner.start(plan)


def test_zero_motion_plan_finishes_on_first_tick():
    plan = NavigationPlanner().plan_click_to_go(document(), Pose(0.0, 0.0, 0.0), Point2D(0.0, 0.0))
    runner = SimulatedNavigationRunner(plan.start_pose)
    runner.start(plan)
    snapshot = runner.tick(0.01)
    assert snapshot.state is SimulationState.FINISHED
    assert snapshot.pose == plan.start_pose
