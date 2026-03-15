import unittest

from usv_uav_marine_coverage.environment import build_default_sea_map, build_obstacle_layout


class EnvironmentTestCase(unittest.TestCase):
    def test_default_sea_map_matches_discussion_notes(self) -> None:
        sea_map = build_default_sea_map()

        self.assertEqual(sea_map.width, 1000.0)
        self.assertEqual(sea_map.height, 1000.0)
        self.assertEqual(sea_map.nearshore.x_start, 0.0)
        self.assertEqual(sea_map.nearshore.x_end, 250.0)
        self.assertEqual(sea_map.risk.x_start, 250.0)
        self.assertEqual(sea_map.risk.x_end, 450.0)
        self.assertEqual(sea_map.offshore.x_start, 450.0)
        self.assertEqual(sea_map.offshore.x_end, 1000.0)

    def test_obstacle_layout_matches_confirmed_constraints(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)

        self.assertEqual(len(layout.risk_zone_obstacles), 4)
        self.assertEqual(len(layout.traversable_corridors), 2)
        self.assertEqual(len(layout.offshore_features), 5)
        self.assertEqual(len(layout.nearshore_monitor_points), 0)
        self.assertEqual(len(layout.offshore_hotspots), 0)
        self.assertTrue(56.0 <= layout.traversable_corridors[0].width <= 72.0)
        self.assertTrue(60.0 <= layout.traversable_corridors[1].width <= 76.0)
        self.assertEqual(len(layout.traversable_corridors[0].control_points), 5)
        self.assertEqual(len(layout.traversable_corridors[1].control_points), 5)
        self.assertLess(
            layout.traversable_corridors[0].y_max,
            layout.traversable_corridors[1].y_min,
        )
        self.assertEqual(
            sum(feature.feature_type == "islet" for feature in layout.offshore_features),
            3,
        )
        self.assertEqual(
            sum(feature.feature_type == "risk_area" for feature in layout.offshore_features),
            2,
        )
        self.assertEqual(layout.nearshore_monitor_points, ())
        self.assertEqual(layout.offshore_hotspots, ())
