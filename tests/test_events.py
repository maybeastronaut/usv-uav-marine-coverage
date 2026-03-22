import unittest
from dataclasses import replace

from usv_uav_marine_coverage.agent_model import HealthStatus
from usv_uav_marine_coverage.environment import build_default_sea_map, build_obstacle_layout
from usv_uav_marine_coverage.events import apply_scheduled_events
from usv_uav_marine_coverage.events.event_types import AgentDamageEvent, EventType
from usv_uav_marine_coverage.execution.execution_types import ExecutionStage
from usv_uav_marine_coverage.execution.progress_feedback import build_initial_progress_states
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.simulation.simulation_agent_runtime import (
    advance_agents_one_step,
    build_initial_execution_states,
    build_wreck_zones,
)
from usv_uav_marine_coverage.simulation.simulation_policy import (
    build_demo_agent_states,
    build_patrol_routes,
)
from usv_uav_marine_coverage.simulation.simulation_task_runtime import (
    requeue_tasks_blocked_by_wrecks,
)
from usv_uav_marine_coverage.tasking.partitioning import build_weighted_voronoi_task_partition
from usv_uav_marine_coverage.tasking.task_types import (
    TaskRecord,
    TaskSource,
    TaskStatus,
    TaskType,
)


def _build_grid_map():
    sea_map = build_default_sea_map()
    obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
    return build_grid_map(sea_map, obstacle_layout)


class EventRuntimeTestCase(unittest.TestCase):
    def test_agent_failure_releases_task_and_marks_agent_failed(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        failed_agent = next(agent for agent in agents if agent.agent_id == "USV-2")
        execution_states["USV-2"] = replace(
            execution_states["USV-2"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="baseline-service-1",
        )
        task_records = (
            TaskRecord(
                task_id="baseline-service-1",
                task_type=TaskType.BASELINE_SERVICE,
                source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
                status=TaskStatus.ASSIGNED,
                priority=5,
                target_x=failed_agent.x + 20.0,
                target_y=failed_agent.y,
                target_row=4,
                target_col=4,
                created_step=10,
                assigned_agent_id="USV-2",
            ),
        )

        updated_agents, updated_execution_states, _, updated_tasks, applied, released = (
            apply_scheduled_events(
                step=40,
                scheduled_events=(
                    AgentDamageEvent(
                        step=40,
                        event_type=EventType.AGENT_FAILURE,
                        agent_id="USV-2",
                    ),
                ),
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=task_records,
            )
        )

        failed_state = next(agent for agent in updated_agents if agent.agent_id == "USV-2")
        self.assertFalse(failed_state.is_operational)
        self.assertEqual(failed_state.health_status.value, "failed")
        self.assertEqual(failed_state.max_speed_mps, 0.0)
        self.assertEqual(updated_execution_states["USV-2"].stage, ExecutionStage.FAILED)
        self.assertEqual(updated_tasks[0].status, TaskStatus.REQUEUED)
        self.assertIsNone(updated_tasks[0].assigned_agent_id)
        self.assertEqual(len(applied), 1)
        self.assertEqual(len(released), 1)

    def test_speed_degradation_updates_agent_capability_without_releasing_task(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = replace(
            execution_states["USV-1"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="baseline-service-2",
        )
        task_records = (
            TaskRecord(
                task_id="baseline-service-2",
                task_type=TaskType.BASELINE_SERVICE,
                source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
                status=TaskStatus.ASSIGNED,
                priority=5,
                target_x=180.0,
                target_y=180.0,
                target_row=5,
                target_col=5,
                created_step=12,
                assigned_agent_id="USV-1",
            ),
        )

        updated_agents, updated_execution_states, _, updated_tasks, _, released = (
            apply_scheduled_events(
                step=30,
                scheduled_events=(
                    AgentDamageEvent(
                        step=30,
                        event_type=EventType.SPEED_DEGRADATION,
                        agent_id="USV-1",
                        speed_multiplier=0.5,
                    ),
                ),
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=task_records,
            )
        )

        degraded = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        self.assertEqual(degraded.health_status.value, "degraded")
        self.assertAlmostEqual(degraded.speed_multiplier, 0.5)
        self.assertAlmostEqual(
            degraded.max_speed_mps,
            degraded.platform_profile.max_speed_mps * 0.5,
        )
        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(updated_tasks[0].status, TaskStatus.ASSIGNED)
        self.assertIsNone(updated_execution_states["USV-1"].active_plan)
        self.assertEqual(released, ())

    def test_speed_degradation_releases_uav_resupply_tasks_supported_by_degraded_usv(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        task_records = (
            TaskRecord(
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=160.0,
                target_y=220.0,
                target_row=None,
                target_col=None,
                created_step=12,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
            ),
        )

        _, _, _, updated_tasks, _, released = apply_scheduled_events(
            step=30,
            scheduled_events=(
                AgentDamageEvent(
                    step=30,
                    event_type=EventType.SPEED_DEGRADATION,
                    agent_id="USV-1",
                    speed_multiplier=0.5,
                ),
            ),
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=task_records,
        )

        self.assertEqual(updated_tasks[0].status, TaskStatus.REQUEUED)
        self.assertIsNone(updated_tasks[0].assigned_agent_id)
        self.assertIsNone(updated_tasks[0].support_agent_id)
        self.assertEqual(released[0]["reason"], "degraded_support_usv_requeue")

    def test_turn_rate_degradation_updates_turn_limit(self) -> None:
        agents = build_demo_agent_states()
        updated_agents, _, _, _, _, _ = apply_scheduled_events(
            step=22,
            scheduled_events=(
                AgentDamageEvent(
                    step=22,
                    event_type=EventType.TURN_RATE_DEGRADATION,
                    agent_id="USV-3",
                    turn_rate_multiplier=0.4,
                ),
            ),
            agents=agents,
            execution_states=build_initial_execution_states(agents),
            progress_states=build_initial_progress_states(agents),
            task_records=(),
        )

        degraded = next(agent for agent in updated_agents if agent.agent_id == "USV-3")
        self.assertEqual(degraded.health_status.value, "degraded")
        self.assertAlmostEqual(degraded.turn_rate_multiplier, 0.4)
        self.assertAlmostEqual(
            degraded.max_turn_rate_degps,
            degraded.platform_profile.max_turn_rate_degps * 0.4,
        )

    def test_weighted_voronoi_skips_failed_usv(self) -> None:
        agents = list(build_demo_agent_states())
        agents[1] = replace(
            agents[1],
            health_status=HealthStatus.FAILED,
            is_operational=False,
            max_speed_mps=0.0,
            cruise_speed_mps=0.0,
        )
        partition = build_weighted_voronoi_task_partition(
            TaskRecord(
                task_id="hotspot-confirmation-1",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_INSPECTED,
                status=TaskStatus.PENDING,
                priority=8,
                target_x=720.0,
                target_y=500.0,
                target_row=10,
                target_col=10,
                created_step=1,
            ),
            agents=tuple(agents),
            execution_states=build_initial_execution_states(tuple(agents)),
        )

        self.assertNotIn("USV-2", partition.primary_usv_ids)
        self.assertNotIn("USV-2", partition.secondary_usv_ids)

    def test_advance_agents_one_step_keeps_failed_usv_stationary(self) -> None:
        agents = list(build_demo_agent_states())
        agents[0] = replace(
            agents[0],
            health_status=HealthStatus.FAILED,
            is_operational=False,
            max_speed_mps=0.0,
            cruise_speed_mps=0.0,
        )
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=build_patrol_routes(agents=agents),
            grid_map=_build_grid_map(),
            dt_seconds=1.0,
        )

        before = next(agent for agent in agents if agent.agent_id == "USV-1")
        after = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        self.assertEqual((before.x, before.y), (after.x, after.y))
        self.assertEqual(after.speed_mps, 0.0)
        self.assertEqual(updated_execution_states["USV-1"].stage, ExecutionStage.FAILED)

    def test_requeue_tasks_blocked_by_wrecks_releases_hotspot_confirmation(self) -> None:
        agents = list(build_demo_agent_states())
        agents[1] = replace(
            agents[1],
            x=900.0,
            y=620.0,
            health_status=HealthStatus.FAILED,
            is_operational=False,
            max_speed_mps=0.0,
            cruise_speed_mps=0.0,
        )
        agents = tuple(agents)
        wreck_zones = build_wreck_zones(agents)
        task = TaskRecord(
            task_id="hotspot-confirmation-wreck",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=906.0,
            target_y=624.0,
            target_row=24,
            target_col=36,
            created_step=10,
            assigned_agent_id="USV-3",
        )

        updated_tasks, reassignments = requeue_tasks_blocked_by_wrecks(
            task_records=(task,),
            agents=agents,
            wreck_zones=wreck_zones,
            step=40,
        )

        self.assertEqual(updated_tasks[0].status, TaskStatus.REQUEUED)
        self.assertIsNone(updated_tasks[0].assigned_agent_id)
        self.assertEqual(len(reassignments), 1)
        self.assertEqual(reassignments[0]["reason"], "wreck_keepout")
        self.assertEqual(reassignments[0]["source_agent_id"], "USV-2")

    def test_requeue_tasks_blocked_by_wrecks_does_not_repeat_during_cooldown(self) -> None:
        agents = list(build_demo_agent_states())
        agents[1] = replace(
            agents[1],
            x=900.0,
            y=620.0,
            health_status=HealthStatus.FAILED,
            is_operational=False,
            max_speed_mps=0.0,
            cruise_speed_mps=0.0,
        )
        agents = tuple(agents)
        wreck_zones = build_wreck_zones(agents)
        blocked_task = TaskRecord(
            task_id="hotspot-confirmation-wreck",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.REQUEUED,
            priority=10,
            target_x=906.0,
            target_y=624.0,
            target_row=24,
            target_col=36,
            created_step=10,
            assigned_agent_id=None,
            retry_after_step=9999,
            agent_retry_after_steps=(
                ("USV-1", 9999),
                ("USV-3", 9999),
            ),
        )

        updated_tasks, reassignments = requeue_tasks_blocked_by_wrecks(
            task_records=(blocked_task,),
            agents=agents,
            wreck_zones=wreck_zones,
            step=120,
        )

        self.assertEqual(updated_tasks[0], blocked_task)
        self.assertEqual(reassignments, ())
