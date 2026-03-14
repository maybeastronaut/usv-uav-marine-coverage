import unittest

from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage
from usv_uav_marine_coverage.simulation.simulation_policy import build_demo_agent_states
from usv_uav_marine_coverage.tasking.nearest_agent_allocator import allocate_nearest_agents
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskSource, TaskStatus, TaskType


class TaskingTestCase(unittest.TestCase):
    def test_allocator_assigns_hotspot_confirmation_only_to_usv(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="hotspot-confirmation-10-10",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=180.0,
            target_y=150.0,
            target_row=10,
            target_col=10,
            created_step=5,
        )

        updated_tasks, decisions = allocate_nearest_agents(
            (task,),
            agents=agents,
            execution_states=execution_states,
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(decisions[0].agent_id, "USV-1")

    def test_allocator_keeps_existing_assignment_when_still_valid(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.GO_TO_TASK
                if agent.agent_id == "USV-2"
                else ExecutionStage.PATROL,
                active_task_id="hotspot-confirmation-8-8" if agent.agent_id == "USV-2" else None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="hotspot-confirmation-8-8",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=560.0,
            target_y=610.0,
            target_row=8,
            target_col=8,
            created_step=3,
            assigned_agent_id="USV-2",
        )

        updated_tasks, decisions = allocate_nearest_agents(
            (task,),
            agents=agents,
            execution_states=execution_states,
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-2")
        self.assertEqual(decisions[0].selection_reason, "keep_existing_assignment")


if __name__ == "__main__":
    unittest.main()
