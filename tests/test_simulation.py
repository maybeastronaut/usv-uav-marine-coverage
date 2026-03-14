import json
import tempfile
import unittest
from pathlib import Path

from usv_uav_marine_coverage.simulation import (
    build_simulation_html,
    build_simulation_replay,
    write_simulation_artifacts,
)


class SimulationTestCase(unittest.TestCase):
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
            any(first_positions[agent_id] != last_positions[agent_id] for agent_id in first_positions)
        )

    def test_simulation_html_contains_replay_controls_and_layers(self) -> None:
        replay = build_simulation_replay(seed=20260314, steps=4, dt_seconds=1.0)
        html = build_simulation_html(replay)

        self.assertIn("USV-UAV Simulation Replay", html)
        self.assertIn('aria-label="Simulation label switch"', html)
        self.assertIn('aria-label="Simulation footprint switch"', html)
        self.assertIn("togglePlayback()", html)
        self.assertIn("Trajectory Layer", html)
        self.assertIn("Valid Information Cells", html)
        self.assertIn("Hotspot Knowledge Layer", html)
        self.assertIn("Replay Footprints", html)

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

            step_records = [json.loads(line) for line in event_lines if '"record_type": "step_snapshot"' in line]
            self.assertTrue(step_records)
            self.assertIn("agent_states", step_records[-1])
            self.assertIn("coverage", step_records[-1])
            self.assertIn("task_layer", step_records[-1])
            self.assertIn("path_layer", step_records[-1])
            self.assertIn("execution_layer", step_records[-1])
            self.assertIn("hotspot_chain", step_records[-1])
            self.assertIn("failure_recovery", step_records[-1])
            self.assertIn("task_decisions", step_records[-1]["task_layer"])
            self.assertIn("path_plans", step_records[-1]["path_layer"])

            summary = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
            self.assertIn("simulation", summary)
            self.assertIn("environment", summary)
            self.assertIn("initial_agents", summary)
            self.assertIn("final_metrics", summary)
            self.assertIn("event_totals", summary["final_metrics"])
