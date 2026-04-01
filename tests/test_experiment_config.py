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
    def test_load_experiment_config_reads_coordination_toggle(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "coordination_toggle.toml"
            path.write_text(
                "\n".join(
                    [
                        "[simulation]",
                        "steps = 10",
                        "dt_seconds = 1.0",
                        "",
                        "[scenario]",
                        'name = "baseline_patrol"',
                        "",
                        "[coordination]",
                        "enable_uav_usv_meeting = false",
                        "",
                        "[algorithms]",
                        'task_allocator = "basic_task_allocator"',
                        'usv_path_planner = "astar_path_planner"',
                        'uav_search_planner = "uav_lawnmower_planner"',
                        'execution_policy = "phase_one_execution"',
                    ]
                ),
                encoding="utf-8",
            )

            config = load_experiment_config(path)

        self.assertFalse(config.coordination.enable_uav_usv_meeting)

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
        self.assertEqual(config.algorithms.zone_partition_policy, "baseline_fixed_partition")

    def test_load_soft_partition_experiment_config_reads_new_partition_policy(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_soft_partition.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "cost_aware_centralized_allocator",
        )
        self.assertEqual(config.algorithms.zone_partition_policy, "soft_partition_policy")

    def test_load_aoi_energy_soft_partition_config_reads_new_partition_policy(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_aoi_energy_weighted_voronoi.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "aoi_energy_auction_allocator",
        )
        self.assertEqual(
            config.algorithms.zone_partition_policy,
            "weighted_voronoi_partition_policy",
        )

    def test_load_rho_soft_partition_config_reads_new_partition_policy(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_rho_weighted_voronoi.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "rho_task_allocator",
        )
        self.assertEqual(
            config.algorithms.zone_partition_policy,
            "weighted_voronoi_partition_policy",
        )

    def test_load_weighted_voronoi_config_reads_new_partition_policy(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_weighted_voronoi.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "cost_aware_centralized_allocator",
        )
        self.assertEqual(
            config.algorithms.zone_partition_policy,
            "weighted_voronoi_partition_policy",
        )

    def test_load_aoi_energy_auction_experiment_config_reads_new_allocator_name(self) -> None:
        config = load_experiment_config(
            Path("configs/aoi_energy_auction_allocator.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "aoi_energy_auction_allocator",
        )

    def test_load_rho_experiment_config_reads_new_allocator_name(self) -> None:
        config = load_experiment_config(
            Path("configs/rho_task_allocator.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "rho_task_allocator",
        )

    def test_load_distributed_cbba_experiment_config_reads_new_allocator_name(self) -> None:
        config = load_experiment_config(
            Path("configs/distributed_cbba_allocator.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "distributed_cbba_allocator",
        )
        self.assertEqual(
            config.algorithms.zone_partition_policy,
            "weighted_voronoi_partition_policy",
        )
        self.assertEqual(config.algorithms.distributed_sync_interval_steps, 1)

    def test_load_distributed_cbba_sync5_config_reads_sync_interval(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_distributed_cbba_weighted_voronoi_sync5.toml"),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "distributed_cbba_allocator",
        )
        self.assertEqual(config.algorithms.distributed_sync_interval_steps, 5)

    def test_load_distributed_cbba_range_config_reads_broadcast_range(self) -> None:
        config = load_experiment_config(
            Path(
                "configs/offshore_hotspot_pressure_distributed_cbba_weighted_voronoi_range350.toml"
            ),
        )

        self.assertEqual(
            config.algorithms.task_allocator,
            "distributed_cbba_allocator",
        )
        self.assertEqual(config.algorithms.distributed_broadcast_range_m, 350.0)

    def test_load_experiment_config_reads_distributed_winner_memory_ttl(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "cbba_memory.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "offshore_hotspot_pressure"

[algorithms]
task_allocator = "distributed_cbba_allocator"
zone_partition_policy = "weighted_voronoi_partition_policy"
distributed_sync_interval_steps = 1
distributed_broadcast_range_m = 0.0
distributed_winner_memory_ttl_steps = 6
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        self.assertEqual(config.algorithms.distributed_winner_memory_ttl_steps, 6)

    def test_load_experiment_config_reads_distributed_bundle_length(self) -> None:
        config = load_experiment_config(
            Path(
                "configs/distributed_overlap_pressure_distributed_cbba_weighted_voronoi_bundle2.toml"
            ),
        )

        self.assertEqual(config.algorithms.task_allocator, "distributed_cbba_allocator")
        self.assertEqual(config.algorithms.distributed_bundle_length, 2)

    def test_load_experiment_config_reads_local_mpc_execution_policy(self) -> None:
        config = load_experiment_config(
            Path("configs/local_mpc_execution.toml"),
        )

        self.assertEqual(config.algorithms.execution_policy, "local_mpc_execution")

    def test_load_experiment_config_reads_scheduled_usv_damage_events(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_failure_event.toml"),
        )

        self.assertEqual(len(config.events), 2)
        self.assertEqual(config.events[0].event_type.value, "agent_failure")
        self.assertEqual(config.events[0].agent_id, "USV-2")
        self.assertEqual(config.events[1].event_type.value, "speed_degradation")
        self.assertEqual(config.events[1].speed_multiplier, 0.5)

    def test_load_experiment_config_reads_agent_failure_only_config(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_agent_failure.toml"),
        )

        self.assertEqual(len(config.events), 1)
        self.assertEqual(config.events[0].event_type.value, "agent_failure")
        self.assertEqual(config.events[0].agent_id, "USV-2")

    def test_load_experiment_config_reads_speed_degradation_only_config(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_speed_degradation.toml"),
        )

        self.assertEqual(len(config.events), 1)
        self.assertEqual(config.events[0].event_type.value, "speed_degradation")
        self.assertEqual(config.events[0].agent_id, "USV-1")
        self.assertEqual(config.events[0].speed_multiplier, 0.5)

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

    def test_load_experiment_config_can_apply_planner_path_stress_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "planner_path_stress.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "planner_path_stress"

[algorithms]
task_allocator = "cost_aware_centralized_allocator"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        self.assertEqual(config.scenario.name, "planner_path_stress")
        self.assertEqual(config.algorithms.task_allocator, "cost_aware_centralized_allocator")
        self.assertEqual(config.information_map.max_active_hotspots, 10)
        self.assertEqual(config.information_map.offshore_hotspot_spawn_probability, 0.055)
        self.assertEqual(config.information_map.baseline_respawn_cooldown_steps, 360)

    def test_load_experiment_config_can_apply_return_to_patrol_stress_scenario(self) -> None:
        config = load_experiment_config(
            Path("configs/return_to_patrol_stress_cost_aware.toml"),
        )

        self.assertEqual(config.scenario.name, "return_to_patrol_stress")
        self.assertEqual(config.algorithms.task_allocator, "cost_aware_centralized_allocator")
        self.assertEqual(config.information_map.max_active_hotspots, 8)
        self.assertEqual(config.information_map.offshore_hotspot_spawn_probability, 0.05)
        self.assertEqual(config.information_map.nearshore_baseline_spawn_probability, 0.00002)
        self.assertEqual(config.information_map.baseline_respawn_cooldown_steps, 420)
        self.assertEqual(config.information_map.information_timeout_steps, 360)

    def test_load_experiment_config_can_apply_aoi_revisit_pressure_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "aoi_revisit.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "aoi_revisit_pressure"

[algorithms]
task_allocator = "aoi_energy_auction_allocator"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        self.assertEqual(config.scenario.name, "aoi_revisit_pressure")
        self.assertEqual(config.algorithms.task_allocator, "aoi_energy_auction_allocator")
        self.assertEqual(config.information_map.information_timeout_steps, 300)
        self.assertEqual(config.information_map.nearshore_information_timeout_steps, 520)
        self.assertEqual(config.information_map.max_active_hotspots, 10)
        self.assertEqual(config.information_map.max_active_baseline_tasks, 2)
        self.assertEqual(config.information_map.nearshore_baseline_spawn_probability, 0.00018)
        self.assertEqual(config.information_map.offshore_hotspot_spawn_probability, 0.05)

    def test_load_experiment_config_can_apply_distributed_overlap_pressure_scenario(
        self,
    ) -> None:
        config = load_experiment_config(
            Path("configs/distributed_overlap_pressure_distributed_cbba_weighted_voronoi.toml"),
        )

        self.assertEqual(config.scenario.name, "distributed_overlap_pressure")
        self.assertEqual(config.algorithms.task_allocator, "distributed_cbba_allocator")
        self.assertEqual(config.information_map.information_timeout_steps, 340)
        self.assertEqual(config.information_map.nearshore_information_timeout_steps, 560)
        self.assertEqual(config.information_map.max_active_baseline_tasks, 2)
        self.assertEqual(config.information_map.max_active_hotspots, 14)
        self.assertEqual(config.information_map.nearshore_baseline_spawn_probability, 0.00035)
        self.assertEqual(config.information_map.nearshore_hotspot_spawn_probability, 0.008)
        self.assertEqual(config.information_map.offshore_hotspot_spawn_probability, 0.075)
        self.assertEqual(config.information_map.hotspot_clearance_cells, 0)

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
        self.assertIn("coordination", payload)
        self.assertIn("algorithms", payload)
        self.assertIn("information_map", payload)
        self.assertEqual(payload["algorithms"]["task_allocator"], "basic_task_allocator")
        self.assertEqual(payload["scenario"]["name"], "baseline_patrol")

    def test_validate_experiment_config_accepts_cost_aware_allocator(self) -> None:
        config = load_experiment_config(Path("configs/cost_aware_allocator.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_soft_partition_policy(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_soft_partition.toml")
        )

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_backlog_aware_partition_policy(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "backlog_partition.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[algorithms]
task_allocator = "cost_aware_centralized_allocator"
zone_partition_policy = "backlog_aware_partition_policy"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_weighted_voronoi_partition_policy(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_weighted_voronoi.toml")
        )

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_failure_hotspot_soft_partition_policy(
        self,
    ) -> None:
        config = load_experiment_config(
            Path("configs/baseline_patrol_rho_failure_hotspot_first_soft_partition.toml")
        )

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_aoi_energy_auction_allocator(self) -> None:
        config = load_experiment_config(Path("configs/aoi_energy_auction_allocator.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_rho_allocator(self) -> None:
        config = load_experiment_config(Path("configs/rho_task_allocator.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_distributed_cbba_allocator(self) -> None:
        config = load_experiment_config(Path("configs/distributed_cbba_allocator.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_scheduled_usv_damage_events(self) -> None:
        config = load_experiment_config(
            Path("configs/offshore_hotspot_pressure_cost_aware_failure_event.toml")
        )

        validate_experiment_config(config)

    def test_validate_experiment_config_rejects_non_usv_damage_event(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "invalid_event.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "offshore_hotspot_pressure"

[algorithms]
task_allocator = "cost_aware_centralized_allocator"
zone_partition_policy = "weighted_voronoi_partition_policy"
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"

[[events]]
step = 3
type = "agent_failure"
agent_id = "UAV-1"
                """.strip(),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        with self.assertRaisesRegex(ValueError, "only supports USV events"):
            validate_experiment_config(config)

    def test_validate_experiment_config_rejects_non_positive_distributed_sync_interval(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "invalid_cbba.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "offshore_hotspot_pressure"

[algorithms]
task_allocator = "distributed_cbba_allocator"
zone_partition_policy = "weighted_voronoi_partition_policy"
distributed_sync_interval_steps = 0
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "distributed_sync_interval_steps must be >= 1"):
                load_experiment_config(path)

    def test_validate_experiment_config_rejects_negative_broadcast_range(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "invalid_cbba_range.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "offshore_hotspot_pressure"

[algorithms]
task_allocator = "distributed_cbba_allocator"
zone_partition_policy = "weighted_voronoi_partition_policy"
distributed_sync_interval_steps = 1
distributed_broadcast_range_m = -1.0
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "distributed_broadcast_range_m must be >= 0.0"):
                load_experiment_config(path)

    def test_validate_experiment_config_rejects_negative_winner_memory_ttl(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "invalid_cbba_memory.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "offshore_hotspot_pressure"

[algorithms]
task_allocator = "distributed_cbba_allocator"
zone_partition_policy = "weighted_voronoi_partition_policy"
distributed_sync_interval_steps = 1
distributed_broadcast_range_m = 0.0
distributed_winner_memory_ttl_steps = -1
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                ValueError,
                "distributed_winner_memory_ttl_steps must be >= 0",
            ):
                load_experiment_config(path)

    def test_validate_experiment_config_rejects_unsupported_distributed_bundle_length(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "invalid_cbba_bundle.toml"
            path.write_text(
                """
[simulation]
seed = 1
steps = 10
dt_seconds = 1.0

[scenario]
name = "distributed_overlap_pressure"

[algorithms]
task_allocator = "distributed_cbba_allocator"
zone_partition_policy = "weighted_voronoi_partition_policy"
distributed_bundle_length = 3
usv_path_planner = "astar_path_planner"
uav_search_planner = "uav_lawnmower_planner"
execution_policy = "phase_one_execution"
                """.strip(),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        with self.assertRaisesRegex(
            ValueError,
            "Unsupported distributed CBBA bundle length 3; supported: \\[1, 2\\]",
        ):
            validate_experiment_config(config)

    def test_validate_experiment_config_accepts_hybrid_astar_planner(self) -> None:
        config = load_experiment_config(Path("configs/hybrid_astar_baseline.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_astar_smoother_planner(self) -> None:
        config = load_experiment_config(Path("configs/astar_smoother_baseline.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_uav_multi_region_planner(self) -> None:
        config = load_experiment_config(Path("configs/uav_multi_region_coverage.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_uav_persistent_multi_region_planner(self) -> None:
        config = load_experiment_config(Path("configs/uav_persistent_multi_region_coverage.toml"))

        validate_experiment_config(config)

    def test_validate_experiment_config_accepts_cost_aware_persistent_combo(self) -> None:
        config = load_experiment_config(Path("configs/cost_aware_uav_persistent_multi_region.toml"))

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
