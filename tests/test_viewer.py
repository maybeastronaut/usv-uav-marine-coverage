import unittest

from usv_uav_marine_coverage.environment import build_default_sea_map, build_obstacle_layout
from usv_uav_marine_coverage.viewer import build_map_html


class ViewerTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.sea_map = build_default_sea_map()
        self.layout = build_obstacle_layout(self.sea_map, seed=20260314)

    def test_clean_mode_initializes_single_html_toggle_view(self) -> None:
        html = build_map_html(self.sea_map, self.layout)

        self.assertIn("USV-UAV Sea Map", html)
        self.assertIn("X Axis (0 m to 1000 m)", html)
        self.assertIn("Y Axis (m)", html)
        self.assertIn('data-view-mode="clean"', html)
        self.assertIn('aria-label="Sea map mode switch"', html)
        self.assertIn('aria-label="Agent label switch"', html)
        self.assertIn("setViewMode('clean')", html)
        self.assertIn("setViewMode('debug')", html)
        self.assertIn("setLabelVisibility(true)", html)
        self.assertIn("setLabelVisibility(false)", html)
        self.assertIn('.debug-layer', html)
        self.assertIn('.agent-label', html)
        self.assertIn('data-show-labels="true"', html)
        self.assertIn("Reserved Trajectory Layer", html)
        self.assertNotIn("Zone Legend", html)
        self.assertNotIn("Transit Channel", html)
        self.assertIn("Static Coverage Preview", html)
        self.assertIn("Coverage Cell", html)
        self.assertIn("Static Footprint Preview", html)
        self.assertIn("USV-1 Footprint", html)
        self.assertIn("Risk Obstacle 1", html)
        self.assertIn("Offshore Risk Area 1", html)
        self.assertIn("Nearshore Baseline Point 1", html)
        self.assertIn("Offshore Hotspot 1", html)
        self.assertIn("USV-1", html)
        self.assertIn("USV-2", html)
        self.assertIn("USV-3", html)
        self.assertIn("UAV-1", html)
        self.assertIn("UAV-2", html)

    def test_html_can_omit_static_validation_layers_when_requested(self) -> None:
        html = build_map_html(
            self.sea_map,
            self.layout,
            mode="clean",
            show_coverage_preview=False,
            show_footprints=False,
        )

        self.assertNotIn("Static Coverage Preview", html)
        self.assertNotIn("Coverage Cell", html)
        self.assertNotIn("Static Footprint Preview", html)
        self.assertNotIn("USV-1 Footprint", html)
        self.assertNotIn("UAV-1 Footprint", html)
        self.assertIn("Reserved Trajectory Layer", html)

    def test_debug_mode_sets_initial_state_to_debug(self) -> None:
        html = build_map_html(self.sea_map, self.layout, mode="debug")

        self.assertIn('data-view-mode="debug"', html)
        self.assertIn('aria-pressed="true"', html)
        self.assertIn('data-show-labels="true"', html)
        self.assertIn("Static Coverage Preview", html)
        self.assertIn("Coverage Cell", html)
        self.assertIn("Static Footprint Preview", html)
        self.assertIn("USV-1 Footprint", html)
        self.assertIn("UAV-1 Footprint", html)
        self.assertIn("Reserved Trajectory Layer", html)
        self.assertIn("USV-1", html)
        self.assertIn("UAV-1", html)
