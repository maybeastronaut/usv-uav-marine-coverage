import unittest

from usv_uav_2_0.agent_model import (
    AgentState,
    AgentTaskState,
    TaskMode,
    UAV_COVERAGE_RADIUS_M,
    USV_COVERAGE_RADIUS_M,
    advance_agent_towards_task,
    assign_agent_task,
    can_cover_point,
    can_detect_point,
    default_coverage_radius,
)


class AgentModelTestCase(unittest.TestCase):
    def test_assign_agent_task_updates_basic_task_state(self) -> None:
        agent = AgentState(
            agent_id="UAV-1",
            kind="UAV",
            x=100.0,
            y=120.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=18.0,
            detection_radius=110.0,
            coverage_radius=70.0,
            task=AgentTaskState(),
        )

        updated = assign_agent_task(agent, TaskMode.INVESTIGATE, target_x=320.0, target_y=440.0)

        self.assertEqual(updated.task.mode, TaskMode.INVESTIGATE)
        self.assertEqual(updated.task.target_x, 320.0)
        self.assertEqual(updated.task.target_y, 440.0)

    def test_advance_agent_towards_task_uses_simple_kinematics(self) -> None:
        agent = AgentState(
            agent_id="USV-1",
            kind="USV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=10.0,
            detection_radius=60.0,
            coverage_radius=35.0,
            task=AgentTaskState(mode=TaskMode.PATROL, target_x=30.0, target_y=40.0),
        )

        advanced = advance_agent_towards_task(agent, dt_seconds=2.0)

        self.assertAlmostEqual(advanced.x, 12.0)
        self.assertAlmostEqual(advanced.y, 16.0)
        self.assertAlmostEqual(advanced.heading_deg, 53.13010235415595)
        self.assertAlmostEqual(advanced.speed_mps, 10.0)
        self.assertTrue(advanced.task.has_target)

    def test_advance_agent_clears_target_when_it_reaches_destination(self) -> None:
        agent = AgentState(
            agent_id="UAV-2",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=12.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(mode=TaskMode.CONFIRM, target_x=6.0, target_y=8.0),
        )

        arrived = advance_agent_towards_task(agent, dt_seconds=1.0)
        stopped = advance_agent_towards_task(arrived, dt_seconds=1.0)

        self.assertAlmostEqual(arrived.x, 6.0)
        self.assertAlmostEqual(arrived.y, 8.0)
        self.assertFalse(arrived.task.has_target)
        self.assertEqual(arrived.task.mode, TaskMode.CONFIRM)
        self.assertEqual(stopped.speed_mps, 0.0)

    def test_detection_and_coverage_radii_are_distinguished(self) -> None:
        agent = AgentState(
            agent_id="USV-2",
            kind="USV",
            x=100.0,
            y=100.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=6.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(),
        )

        self.assertTrue(can_detect_point(agent, 145.0, 100.0))
        self.assertFalse(can_cover_point(agent, 145.0, 100.0))
        self.assertFalse(can_detect_point(agent, 180.0, 100.0))

    def test_default_coverage_radius_matches_confirmed_footprint_values(self) -> None:
        self.assertEqual(default_coverage_radius("USV"), USV_COVERAGE_RADIUS_M)
        self.assertEqual(default_coverage_radius("UAV"), UAV_COVERAGE_RADIUS_M)
        self.assertEqual(USV_COVERAGE_RADIUS_M, 50.0)
        self.assertEqual(UAV_COVERAGE_RADIUS_M, 100.0)
