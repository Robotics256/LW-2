"""Check the launch defaults and controller contract delivered to ROS."""

import argparse
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "SCRIPTS"))
from navigation_modes import add_mode_arguments, add_trail_arguments, behavior_tree


class NavigationProfileTests(unittest.TestCase):
    def test_cli_defaults_and_opt_in_orientation(self):
        parser = argparse.ArgumentParser()
        add_mode_arguments(parser)
        add_trail_arguments(parser)
        self.assertEqual(parser.parse_args([]).navigation_mode, "proximity")
        self.assertIsNone(parser.parse_args([]).trail)
        self.assertEqual(parser.parse_args(["--orientation"]).navigation_mode, "orientation")
        self.assertEqual(parser.parse_args(["--mode", "position"]).navigation_mode, "position")
        self.assertEqual(parser.parse_args(["--ignore-orientation"]).navigation_mode, "position")
        self.assertEqual(parser.parse_args(["--mode", "proximity"]).navigation_mode, "proximity")
        self.assertIs(parser.parse_args(["--no-trail"]).trail, False)

    def test_every_tree_selects_an_installed_checker_for_every_robot_profile(self):
        package = ROOT / "ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo"
        for model in ("burger", "waffle", "waffle_pi"):
            profile = yaml.safe_load((package / f"params/warehouse_{model}.yaml").read_text())
            controller = profile["controller_server"]["ros__parameters"]
            for mode in ("position", "orientation", "proximity"):
                tree = ET.parse(behavior_tree(mode))
                selector = tree.find(".//GoalCheckerSelector")
                checker = (selector.get("default_goal_checker") if selector is not None
                           else tree.find(".//FollowPath").get("goal_checker_id"))
                self.assertIn(checker, controller["goal_checker_plugins"])
                self.assertEqual(controller[checker]["xy_goal_tolerance"],
                                 0.3 if mode == "proximity" else 0.03)

    def test_automotive_world_and_map_are_self_contained(self):
        package = ROOT / "ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo"
        world = ET.parse(package / "worlds/turtlebot3_warehouse_automotive.world")
        self.assertFalse(world.findall(".//uri"))
        self.assertIsNotNone(world.find("world/plugin[@name='gazebo_ros_state']"))
        path = package / "maps/warehouse_automotive.yaml"
        config = yaml.safe_load(path.read_text())
        self.assertTrue((path.parent / config["image"]).is_file())


if __name__ == "__main__":
    unittest.main()
