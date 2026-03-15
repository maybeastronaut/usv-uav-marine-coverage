import unittest

from usv_uav_marine_coverage.agent_model import (
    UAV_COVERAGE_RADIUS_M,
    UAV_RECHARGE_RELEASE_RATIO,
    UAV_RESUPPLY_TRIGGER_MARGIN,
    USV_COVERAGE_RADIUS_M,
    AgentState,
    AgentTaskState,
    TaskMode,
    advance_agent_towards_task,
    assign_agent_task,
    can_cover_point,
    can_detect_point,
    default_coverage_radius,
    default_platform_profile,
    energy_ratio,
    estimate_uav_energy_to_point,
    has_completed_uav_resupply,
    needs_uav_resupply,
    recharge_agent_energy,
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
            max_speed_mps=0.0,
            detection_radius=110.0,
            coverage_radius=70.0,
            task=AgentTaskState(),
        )

        updated = assign_agent_task(agent, TaskMode.INVESTIGATE, target_x=320.0, target_y=440.0)

        self.assertEqual(updated.task.mode, TaskMode.INVESTIGATE)
        self.assertEqual(updated.task.target_x, 320.0)
        self.assertEqual(updated.task.target_y, 440.0)

    def test_uav_closed_loop_update_respects_speed_and_turn_limits(self) -> None:
        profile = default_platform_profile("UAV")
        agent = AgentState(
            agent_id="UAV-1",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=180.0,
            speed_mps=0.0,
            max_speed_mps=profile.max_speed_mps,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(mode=TaskMode.PATROL, target_x=100.0, target_y=0.0),
            max_acceleration_mps2=profile.max_acceleration_mps2,
            max_deceleration_mps2=profile.max_deceleration_mps2,
            max_turn_rate_degps=profile.max_turn_rate_degps,
            cruise_speed_mps=profile.cruise_speed_mps,
            arrival_tolerance_m=profile.arrival_tolerance_m,
        )

        advanced = advance_agent_towards_task(agent, dt_seconds=1.0)

        self.assertGreater(advanced.x, 0.0)
        self.assertLess(abs(advanced.turn_rate_degps), profile.max_turn_rate_degps + 1e-9)
        self.assertLessEqual(advanced.speed_mps, profile.max_acceleration_mps2 + 1e-9)
        self.assertNotEqual(advanced.heading_deg, 0.0)
        self.assertTrue(advanced.task.has_target)

    def test_usv_turns_more_slowly_than_uav_for_same_target(self) -> None:
        uav = AgentState(
            agent_id="UAV-1",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=90.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(mode=TaskMode.PATROL, target_x=100.0, target_y=0.0),
        )
        usv = AgentState(
            agent_id="USV-1",
            kind="USV",
            x=0.0,
            y=0.0,
            heading_deg=90.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(mode=TaskMode.PATROL, target_x=100.0, target_y=0.0),
        )

        advanced_uav = advance_agent_towards_task(uav, dt_seconds=1.0)
        advanced_usv = advance_agent_towards_task(usv, dt_seconds=1.0)

        self.assertGreater(abs(advanced_uav.turn_rate_degps), abs(advanced_usv.turn_rate_degps))
        self.assertGreater(advanced_uav.speed_mps, advanced_usv.speed_mps)
        self.assertGreater(advanced_uav.x, advanced_usv.x)

    def test_usv_needs_multiple_steps_to_realign_for_side_target(self) -> None:
        usv = AgentState(
            agent_id="USV-2",
            kind="USV",
            x=0.0,
            y=0.0,
            heading_deg=90.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(mode=TaskMode.PATROL, target_x=100.0, target_y=0.0),
        )

        step_1 = advance_agent_towards_task(usv, dt_seconds=1.0)
        step_2 = advance_agent_towards_task(step_1, dt_seconds=1.0)
        step_3 = advance_agent_towards_task(step_2, dt_seconds=1.0)

        self.assertNotEqual(step_1.heading_deg, 0.0)
        self.assertTrue(step_1.task.has_target)
        self.assertLess(abs(step_3.heading_deg), abs(step_1.heading_deg))
        self.assertGreater(step_3.x, step_1.x)

    def test_usv_keeps_reasonable_forward_speed_for_moderate_heading_error(self) -> None:
        usv = AgentState(
            agent_id="USV-2",
            kind="USV",
            x=0.0,
            y=0.0,
            heading_deg=25.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(mode=TaskMode.PATROL, target_x=120.0, target_y=0.0),
        )

        step_1 = advance_agent_towards_task(usv, dt_seconds=1.0)
        step_2 = advance_agent_towards_task(step_1, dt_seconds=1.0)

        self.assertGreater(step_2.speed_mps, 2.5)

    def test_arrival_clears_target_and_then_agent_decelerates_gradually(self) -> None:
        agent = AgentState(
            agent_id="USV-3",
            kind="USV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=4.0,
            max_speed_mps=8.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(mode=TaskMode.CONFIRM, target_x=2.0, target_y=0.0),
            max_acceleration_mps2=1.8,
            max_deceleration_mps2=2.0,
            max_turn_rate_degps=28.0,
            cruise_speed_mps=5.5,
            arrival_tolerance_m=5.0,
        )

        arrived = advance_agent_towards_task(agent, dt_seconds=1.0)
        stopped = advance_agent_towards_task(arrived, dt_seconds=1.0)

        self.assertFalse(arrived.task.has_target)
        self.assertGreater(arrived.x, 0.0)
        self.assertAlmostEqual(arrived.speed_mps, 2.0)
        self.assertAlmostEqual(stopped.speed_mps, 0.0)

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

    def test_uav_energy_decreases_during_closed_loop_motion(self) -> None:
        agent = AgentState(
            agent_id="UAV-1",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(mode=TaskMode.INVESTIGATE, target_x=300.0, target_y=0.0),
        )

        advanced = advance_agent_towards_task(agent, dt_seconds=1.0)

        self.assertLess(advanced.energy_level, agent.energy_level)
        self.assertLess(energy_ratio(advanced), 1.0)

    def test_recharge_agent_energy_restores_uav_energy_and_caps_at_capacity(self) -> None:
        agent = AgentState(
            agent_id="UAV-2",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
            energy_level=20.0,
        )

        recharged = recharge_agent_energy(agent, dt_seconds=10.0)

        self.assertEqual(recharged.energy_level, 140.0)

    def test_needs_uav_resupply_triggers_below_reserve_threshold(self) -> None:
        agent = AgentState(
            agent_id="UAV-3",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
            energy_level=40.0,
        )

        self.assertTrue(needs_uav_resupply(agent))

    def test_estimate_uav_energy_to_point_includes_trigger_margin(self) -> None:
        agent = AgentState(
            agent_id="UAV-4",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
        )

        estimated_energy = estimate_uav_energy_to_point(agent, 160.0, 0.0)

        self.assertGreaterEqual(
            estimated_energy,
            agent.energy_capacity * agent.minimum_reserve_ratio + UAV_RESUPPLY_TRIGGER_MARGIN,
        )

    def test_has_completed_uav_resupply_triggers_at_ninety_percent(self) -> None:
        agent = AgentState(
            agent_id="UAV-5",
            kind="UAV",
            x=0.0,
            y=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
            energy_level=180.0 * UAV_RECHARGE_RELEASE_RATIO,
        )

        self.assertTrue(has_completed_uav_resupply(agent))
