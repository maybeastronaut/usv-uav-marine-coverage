import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from usv_uav_marine_coverage.simulation import write_simulation_artifacts
from usv_uav_marine_coverage.simulation.experiment_batch import (
    load_batch_experiment_spec,
    run_batch_experiment,
)


class ExperimentBatchTestCase(unittest.TestCase):
    def test_load_batch_experiment_spec_reads_runs(self) -> None:
        spec = load_batch_experiment_spec(Path("configs/phase_one_batch.toml"))

        self.assertEqual(spec.name, "phase_one_baseline_batch")
        self.assertFalse(spec.generate_html)
        self.assertEqual(len(spec.runs), 2)
        self.assertEqual(spec.runs[0].label, "baseline_seed_20260314")

    def test_run_batch_experiment_writes_aggregate_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            base_config = temp_root / "baseline.toml"
            base_config.write_text(
                """
[simulation]
seed = 20260314
steps = 4
dt_seconds = 1.0

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
            batch_config = temp_root / "batch.toml"
            batch_config.write_text(
                f"""
[batch]
name = "smoke_batch"
base_config = "{base_config.name}"
output_dir = "{(temp_root / "outputs").name}"
generate_html = false

[[runs]]
label = "seed_a"
seed = 20260314
steps = 4

[[runs]]
label = "seed_b"
seed = 20260314
steps = 4
                """.strip(),
                encoding="utf-8",
            )

            spec = load_batch_experiment_spec(batch_config)
            artifacts = run_batch_experiment(spec)

            self.assertTrue(artifacts.runs_path.exists())
            self.assertTrue(artifacts.summary_path.exists())

            run_records = artifacts.runs_path.read_text(encoding="utf-8").strip().splitlines()
            self.assertEqual(len(run_records), 2)

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["batch"]["name"], "smoke_batch")
            self.assertEqual(summary["batch"]["run_count"], 2)
            self.assertIn("aggregates", summary)
            self.assertIn("coverage_ratio", summary["aggregates"])
            self.assertEqual(summary["runs"][0]["label"], "seed_a")
            self.assertEqual(summary["batch"]["successful_runs"], 2)
            self.assertEqual(summary["batch"]["failed_runs"], 0)

    def test_run_batch_experiment_records_failed_run_and_continues(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            base_config = temp_root / "baseline.toml"
            base_config.write_text(
                """
[simulation]
seed = 20260314
steps = 4
dt_seconds = 1.0

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
            batch_config = temp_root / "batch.toml"
            batch_config.write_text(
                f"""
[batch]
name = "failure_batch"
base_config = "{base_config.name}"
output_dir = "{(temp_root / "outputs").name}"
generate_html = false

[[runs]]
label = "good"
seed = 20260314
steps = 4

[[runs]]
label = "bad"
seed = 20260315
steps = 4
                """.strip(),
                encoding="utf-8",
            )

            spec = load_batch_experiment_spec(batch_config)
            original_write_simulation_artifacts = write_simulation_artifacts

            def fail_one_seed(*args, **kwargs):  # type: ignore[no-untyped-def]
                if kwargs.get("seed") == 20260315:
                    raise ValueError("forced batch failure")
                return original_write_simulation_artifacts(*args, **kwargs)

            with patch(
                "usv_uav_marine_coverage.simulation.write_simulation_artifacts",
                side_effect=fail_one_seed,
            ):
                artifacts = run_batch_experiment(spec)
            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))

            self.assertEqual(summary["batch"]["successful_runs"], 1)
            self.assertEqual(summary["batch"]["failed_runs"], 1)
            failed_run = next(run for run in summary["runs"] if run["label"] == "bad")
            self.assertEqual(failed_run["status"], "failed")
            self.assertEqual(failed_run["error_type"], "ValueError")
