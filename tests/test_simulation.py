import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    ExecutionStage,
)
from usv_uav_marine_coverage.simulation import (
    build_simulation_html,
    build_simulation_replay,
    write_simulation_artifacts,
)
from usv_uav_marine_coverage.simulation.simulation_core import _confirmation_indices_for_usv
from usv_uav_marine_coverage.simulation.simulation_logging import build_event_totals
from usv_uav_marine_coverage.simulation.simulation_policy import build_demo_agent_states
from usv_uav_marine_coverage.simulation.simulation_replay_view import _build_frame_summaries
from usv_uav_marine_coverage.tasking.task_types import (
    TaskRecord,
    TaskSource,
    TaskStatus,
    TaskType,
)


class SimulationTestCase(unittest.TestCase):
    def test_demo_agents_start_on_left_side_of_map(self) -> None:
        agents = build_demo_agent_states()

        self.assertTrue(all(agent.x < 250.0 for agent in agents))

    def test_replay_contains_multiple_frames_and_agent_motion(self) -> None:
        replay = build_simulation_replay(seed=20260314, steps=6, dt_seconds=1.0)

        self.assertEqual(len(replay.frames), 7)
        first_frame = replay.frames[0]
        last_frame = replay.frames[-1]
        self.assertEqual(first_frame.step, 0)
        self.assertEqual(last_frame.step, 6)

        first_positions = {agent.agent_id: (agent.x, agent.y) for agent in first_frame.agents}
        last_positions = {agent.agent_id: (agent.x, agent.y) for agent in last_frame.agents}
        self.assertTrue(
            any(
                first_positions[agent_id] != last_positions[agent_id]
                for agent_id in first_positions
            )
        )

    def test_replay_valid_and_stale_counts_ignore_obstacle_cells(self) -> None:
        replay = build_simulation_replay(seed=20260314, steps=2, dt_seconds=1.0)
        final_frame = replay.frames[-1]

        self.assertEqual(
            len(final_frame.valid_cells) + len(final_frame.stale_cells),
            replay.step_logs[-1]["coverage"]["total_coverable_cells"],
        )

    def test_seeded_hotspots_do_not_create_confirmation_tasks_at_step_zero(self) -> None:
        replay = build_simulation_replay(seed=20260314, steps=1, dt_seconds=1.0)

        hotspot_tasks = [
            task
            for task in replay.step_logs[0]["task_layer"]["tasks"]
            if task["task_type"] == "hotspot_confirmation"
        ]

        self.assertEqual(hotspot_tasks, [])

    def test_simulation_html_contains_replay_controls_and_layers(self) -> None:
        replay = build_simulation_replay(seed=20260314, steps=4, dt_seconds=1.0)
        html = build_simulation_html(replay)

        self.assertIn("USV-UAV Simulation Replay", html)
        self.assertIn('aria-label="Simulation label switch"', html)
        self.assertIn('aria-label="Simulation footprint switch"', html)
        self.assertIn("togglePlayback()", html)
        self.assertIn("frameRenderData", html)
        self.assertIn("plannedPathData", html)
        self.assertIn("interpolateUsvPose", html)
        self.assertIn("hermitePoint", html)
        self.assertIn('id="trajectoryLayer"', html)
        self.assertIn("Planned Path Layer", html)
        self.assertIn('stroke-dasharray="8 6"', html)
        self.assertIn("Valid Information Cells", html)
        self.assertIn("Stale Information Cells", html)
        self.assertIn("Baseline Task Layer", html)
        self.assertIn("Hotspot Knowledge Layer", html)
        self.assertIn("Replay Footprints", html)
        self.assertIn('id="staleInfoLayer"', html)
        self.assertIn('id="staleCells"', html)
        self.assertIn("UAV Checked Marks", html)
        self.assertIn("Confirmed Hotspots", html)

    def test_replay_frame_summaries_use_cumulative_hotspot_event_totals(self) -> None:
        replay = build_simulation_replay(seed=20260325, steps=12, dt_seconds=1.0)

        summaries = json.loads(_build_frame_summaries(replay))

        self.assertEqual(len(summaries), len(replay.frames))
        self.assertEqual(summaries[0]["uav_checked_marks"], 0)
        self.assertEqual(summaries[0]["confirmed_hotspots"], 0)
        self.assertGreaterEqual(summaries[0]["active_hotspots"], 0)

        final_totals = build_event_totals(replay.step_logs)
        self.assertEqual(summaries[-1]["uav_checked_marks"], final_totals["uav_checked_marks"])
        self.assertEqual(summaries[-1]["confirmed_hotspots"], final_totals["confirmed_hotspots"])

    def test_simulation_artifacts_include_machine_readable_logs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                seed=20260314,
                steps=5,
                dt_seconds=1.0,
            )

            self.assertTrue(artifacts.html_path.exists())
            self.assertTrue(artifacts.events_path.exists())
            self.assertTrue(artifacts.summary_path.exists())

            event_lines = artifacts.events_path.read_text(encoding="utf-8").strip().splitlines()
            self.assertGreaterEqual(len(event_lines), 4)
            first_record = json.loads(event_lines[0])
            self.assertEqual(first_record["record_type"], "simulation_metadata")
            self.assertIn("experiment_config", first_record)
            self.assertEqual(
                first_record["experiment_config"]["algorithms"]["task_allocator"],
                "basic_task_allocator",
            )

            step_records = [
                json.loads(line) for line in event_lines if '"record_type": "step_snapshot"' in line
            ]
            self.assertTrue(step_records)
            self.assertIn("agent_states", step_records[-1])
            self.assertIn("coverage", step_records[-1])
            self.assertIn("task_layer", step_records[-1])
            self.assertIn("path_layer", step_records[-1])
            self.assertIn("execution_layer", step_records[-1])
            self.assertIn("hotspot_chain", step_records[-1])
            self.assertIn("failure_recovery", step_records[-1])
            self.assertIn("task_decisions", step_records[-1]["task_layer"])
            self.assertIn("tasks", step_records[-1]["task_layer"])
            self.assertIn("retry_after_step", step_records[-1]["task_layer"]["tasks"][0])
            self.assertIn("agent_retry_after_steps", step_records[-1]["task_layer"]["tasks"][0])
            if step_records[-1]["task_layer"]["task_decisions"]:
                self.assertIn(
                    "selection_details",
                    step_records[-1]["task_layer"]["task_decisions"][0],
                )
            self.assertIn("path_plans", step_records[-1]["path_layer"])
            self.assertIn("planner_metrics", step_records[-1]["path_layer"])
            self.assertIn("planner_name", step_records[-1]["path_layer"]["path_plans"][0])
            self.assertIn("total_calls", step_records[-1]["path_layer"]["planner_metrics"])
            self.assertIn("expanded_nodes", step_records[-1]["path_layer"]["planner_metrics"])
            self.assertIn("by_context", step_records[-1]["path_layer"]["planner_metrics"])
            self.assertIn(
                "execution_stage", step_records[-1]["execution_layer"]["tracking_updates"][0]
            )
            self.assertIn(
                "last_return_plan_step",
                step_records[-1]["execution_layer"]["tracking_updates"][0],
            )
            self.assertIn(
                "last_patrol_plan_step",
                step_records[-1]["execution_layer"]["tracking_updates"][0],
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertIn("simulation", summary)
            self.assertIn("environment", summary)
            self.assertIn("initial_agents", summary)
            self.assertIn("final_metrics", summary)
            self.assertIn("experiment_config", summary["simulation"])
            self.assertIn("stale_cells", summary["final_metrics"])
            self.assertIn("event_totals", summary["final_metrics"])
            self.assertIn("astar_total_calls", summary["final_metrics"]["event_totals"])
            self.assertIn("astar_expanded_nodes", summary["final_metrics"]["event_totals"])
            self.assertIn("energy_level", summary["initial_agents"][0])

    def test_simulation_artifacts_can_load_experiment_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "experiment.toml"
            config_path.write_text(
                """
[simulation]
seed = 20260314
steps = 6
dt_seconds = 1.0

[scenario]
name = "mixed_task_pressure"

[algorithms]
task_allocator = "basic_task_allocator"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"

[information_map]
information_timeout_steps = 400
nearshore_information_timeout_steps = 800
                """.strip(),
                encoding="utf-8",
            )
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=config_path,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["simulation"]["seed"], 20260314)
            self.assertEqual(summary["simulation"]["steps"], 6)
            self.assertEqual(
                summary["simulation"]["experiment_config"]["scenario"]["name"],
                "mixed_task_pressure",
            )
            self.assertEqual(
                summary["simulation"]["experiment_config"]["information_map"][
                    "nearshore_information_timeout_steps"
                ],
                800,
            )

    def test_simulation_artifacts_can_skip_html_and_still_write_logs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                seed=20260314,
                steps=5,
                dt_seconds=1.0,
            )

            self.assertIsNone(artifacts.html_path)
            self.assertFalse(output_path.exists())
            self.assertTrue(artifacts.events_path.exists())
            self.assertTrue(artifacts.summary_path.exists())

    def test_simulation_artifacts_can_run_with_cost_aware_allocator_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=Path("configs/cost_aware_allocator.toml"),
                steps=5,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(
                summary["simulation"]["experiment_config"]["algorithms"]["task_allocator"],
                "cost_aware_centralized_allocator",
            )

    def test_simulation_artifacts_can_run_with_aoi_energy_auction_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=Path("configs/aoi_energy_auction_allocator.toml"),
                steps=5,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(
                summary["simulation"]["experiment_config"]["algorithms"]["task_allocator"],
                "aoi_energy_auction_allocator",
            )

    def test_simulation_artifacts_can_run_with_rho_allocator_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=Path("configs/rho_task_allocator.toml"),
                steps=5,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(
                summary["simulation"]["experiment_config"]["algorithms"]["task_allocator"],
                "rho_task_allocator",
            )

    def test_simulation_artifacts_can_run_with_hybrid_astar_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=Path("configs/hybrid_astar_baseline.toml"),
                steps=5,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(
                summary["simulation"]["experiment_config"]["algorithms"]["usv_path_planner"],
                "hybrid_astar_path_planner",
            )

    def test_simulation_artifacts_can_run_with_astar_smoother_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=Path("configs/astar_smoother_baseline.toml"),
                steps=5,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(
                summary["simulation"]["experiment_config"]["algorithms"]["usv_path_planner"],
                "astar_smoother_path_planner",
            )

    def test_simulation_artifacts_can_run_with_uav_multi_region_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=Path("configs/uav_multi_region_coverage.toml"),
                steps=5,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(
                summary["simulation"]["experiment_config"]["algorithms"]["uav_search_planner"],
                "uav_multi_region_coverage_planner",
            )

    def test_simulation_artifacts_can_run_with_uav_persistent_multi_region_config(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "simulation_replay.html"
            artifacts = write_simulation_artifacts(
                output_path=output_path,
                generate_html=False,
                config_path=Path("configs/uav_persistent_multi_region_coverage.toml"),
                steps=5,
            )

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(
                summary["simulation"]["experiment_config"]["algorithms"]["uav_search_planner"],
                "uav_persistent_multi_region_coverage_planner",
            )

    def test_usv_only_confirms_hotspot_while_on_task_at_target_cell(self) -> None:
        execution_states = {
            "USV-1": AgentExecutionState(
                agent_id="USV-1",
                stage=ExecutionStage.ON_TASK,
                active_task_id="hotspot-confirmation-10-8",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-1",
                patrol_waypoint_index=0,
            ),
            "USV-2": AgentExecutionState(
                agent_id="USV-2",
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id="hotspot-confirmation-10-8",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-2",
                patrol_waypoint_index=0,
            ),
        }
        hotspot_task = TaskRecord(
            task_id="hotspot-confirmation-10-8",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.IN_PROGRESS,
            priority=10,
            target_x=212.5,
            target_y=262.5,
            target_row=10,
            target_col=8,
            created_step=2,
            assigned_agent_id="USV-1",
        )
        baseline_task = TaskRecord(
            task_id="baseline-service-10-8",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.IN_PROGRESS,
            priority=1,
            target_x=212.5,
            target_y=262.5,
            target_row=10,
            target_col=8,
            created_step=2,
            assigned_agent_id="USV-1",
        )

        self.assertEqual(
            _confirmation_indices_for_usv(
                agent_id="USV-1",
                observed_indices=((10, 8), (10, 9)),
                task_records=(hotspot_task,),
                execution_states=execution_states,
            ),
            ((10, 8),),
        )
        self.assertEqual(
            _confirmation_indices_for_usv(
                agent_id="USV-2",
                observed_indices=((10, 8),),
                task_records=(replace(hotspot_task, assigned_agent_id="USV-2"),),
                execution_states=execution_states,
            ),
            (),
        )
        self.assertEqual(
            _confirmation_indices_for_usv(
                agent_id="USV-1",
                observed_indices=((10, 8),),
                task_records=(baseline_task,),
                execution_states={"USV-1": execution_states["USV-1"]},
            ),
            (),
        )
