import usv_uav_marine_coverage.agent_model as am
agent = am.AgentState(agent_id="UAV-1", kind="UAV", x=0, y=0, heading_deg=0, speed_mps=0, max_speed_mps=10,
    detection_radius=10, coverage_radius=10, task=am.AgentTaskState(mode=am.TaskMode.PATROL),
    energy_capacity=180.0, energy_level=0.0, minimum_reserve_ratio=0.25, platform_profile=am.default_platform_profile("UAV"), is_operational=True)
print("needs?", am.needs_uav_resupply(agent))
