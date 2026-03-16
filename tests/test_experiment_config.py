import tempfile
import unittest
from pathlib import Path

from usv_uav_marine_coverage.simulation.experiment_config import (
    build_default_experiment_config,
    load_experiment_config,
    serialize_experiment_config,
    validate_experiment_config,
)


class ExperimentConfigTestCase(unittest.TestCase):
    def test_load_experiment_config_reads_toml_sections(self) -> None:
        config = load_experiment_config(
            Path("configs/phase_one_baseline.toml"),
        )

        self.assertEqual(config.simulation.seed, 20260314)
        self.assertEqual(config.simulation.steps, 1200)
        self.assertEqual(config.scenario.name, "baseline_patrol")
        self.assertEqual(config.algorithms.task_allocator, "basic_task_allocator")
        self.assertEqual(config.algorithms.usv_path_planner, "astar_path_planner")
        self.assertEqual(config.information_map.information_timeout_steps, 400)
        self.assertEqual(config.information_map.nearshore_information_timeout_steps, 800)

    def test_load_cost_aware_experiment_config_reads_new_allocator_name(self) -> None:
        config = load_experiment_config(
            Path("configs/cost_aware_allocator.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "cost_aware_centralized_allocator",
        )

    def test_load_experiment_config_can_apply_scenario_preset(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "experiment.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "nearshore_baseline_pressure"

[algorithms]
task_allocator = "basic_task_allocator"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )

            config = load_experiment_config(path)

        self.assertEqual(config.scenario.name, "nearshore_baseline_pressure")
        self.assertEqual(config.information_map.max_active_baseline_tasks, 3)
        self.assertEqual(config.information_map.baseline_respawn_cooldown_steps, 120)

    def test_load_experiment_config_overlays_information_map_on_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "experiment.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "nearshore_baseline_pressure"

[algorithms]
task_allocator = "basic_task_allocator"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"

[information_map]
max_active_baseline_tasks = 2
                """.strip(),
                encoding="utf-8",
            )

            config = load_experiment_config(path)

        self.assertEqual(config.scenario.name, "nearshore_baseline_pressure")
        self.assertEqual(config.information_map.max_active_baseline_tasks, 2)
        self.assertEqual(config.information_map.baseline_respawn_cooldown_steps, 120)

    def test_load_experiment_config_applies_cli_overrides(self) -> None:
        config = load_experiment_config(
            Path("configs/phase_one_baseline.toml"),
            scenario_override="mixed_task_pressure",
            seed_override=20260317,
            steps_override=500,
            dt_override=2.0,
        )

        self.assertEqual(config.scenario.name, "mixed_task_pressure")
        self.assertEqual(config.simulation.seed, 20260317)
        self.assertEqual(config.simulation.steps, 500)
        self.assertEqual(config.simulation.dt_seconds, 2.0)

    def test_validate_experiment_config_rejects_unknown_algorithm_name(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "experiment.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[algorithms]
task_allocator = "unknown_allocator"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"

[information_map]
information_timeout_steps = 400
nearshore_information_timeout_steps = 800
                """.strip(),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        with self.assertRaises(ValueError):
            validate_experiment_config(config)

    def test_serialize_experiment_config_returns_plain_payload(self) -> None:
        payload = serialize_experiment_config(build_default_experiment_config())

        self.assertIn("simulation", payload)
        self.assertIn("scenario", payload)
        self.assertIn("algorithms", payload)
        self.assertIn("information_map", payload)
        self.assertEqual(payload["algorithms"]["task_allocator"], "basic_task_allocator")
        self.assertEqual(payload["scenario"]["name"], "baseline_patrol")

    def test_validate_experiment_config_accepts_cost_aware_allocator(self) -> None:
        config = load_experiment_config(Path("configs/cost_aware_allocator.toml"))

        validate_experiment_config(config)

    def test_load_experiment_config_rejects_unknown_scenario_name(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "experiment.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "unknown_scenario"

[algorithms]
task_allocator = "basic_task_allocator"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                load_experiment_config(path)
