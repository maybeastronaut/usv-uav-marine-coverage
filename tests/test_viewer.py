import unittest

from usv_uav_2_0.environment import build_default_sea_map, build_obstacle_layout
from usv_uav_2_0.viewer import build_map_html


class ViewerTestCase(unittest.TestCase):
    def test_map_html_contains_axes_without_zone_legend(self) -> None:
        sea_map = build_default_sea_map()
        html = build_map_html(sea_map, build_obstacle_layout(sea_map, seed=20260314))

        self.assertIn("USV-UAV Sea Map", html)
        self.assertIn("X Axis (0 m to 1000 m)", html)
        self.assertIn("Y Axis (m)", html)
        self.assertNotIn("Zone Legend", html)
        self.assertNotIn("Transit Channel", html)
        self.assertIn("Risk Obstacle 1", html)
        self.assertIn("Offshore Risk Area 1", html)
        self.assertIn("Nearshore Baseline Point 1", html)
        self.assertIn("Offshore Hotspot 1", html)
        self.assertIn("USV-1", html)
        self.assertIn("USV-2", html)
        self.assertIn("USV-3", html)
        self.assertIn("UAV-1", html)
        self.assertIn("UAV-2", html)
