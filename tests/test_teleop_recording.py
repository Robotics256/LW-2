"""Keyboard speed caps and timestamped waypoint capture regressions."""

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'SCRIPTS'))
from lw2_config import load_teleop_config

try:
    import teleop_keyboard_save as teleop
    from waypoint_capture import capture_waypoint_pose, WaypointCaptureError
    from geometry_msgs.msg import Point, TransformStamped
    from builtin_interfaces.msg import Time as Stamp
except ImportError:
    teleop = None


class TeleopConfigTests(unittest.TestCase):
    def test_default_limits(self):
        self.assertEqual(load_teleop_config(), {'max_linear_speed': .15, 'max_angular_speed': .8})

    def test_configured_limits_and_zero_are_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps({'teleop': {'max_linear_speed': 0, 'max_angular_speed': .4}}))
            self.assertEqual(load_teleop_config(path), {'max_linear_speed': 0., 'max_angular_speed': .4})

    def test_invalid_limits_fail_instead_of_disabling_the_cap(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            for value in (-1, True, '0.1', None, float('nan'), float('inf')):
                with self.subTest(value=value):
                    path.write_text(json.dumps({'teleop': {'max_linear_speed': value, 'max_angular_speed': .8}}))
                    with self.assertRaises(ValueError):
                        load_teleop_config(path)


@unittest.skipIf(teleop is None, 'source ROS setup to test actual message/TF types')
class TeleopRosHelpersTests(unittest.TestCase):
    def test_keyboard_commands_are_capped_in_both_directions_and_by_model(self):
        for model, max_linear, max_angular in (('burger', .22, 2.84), ('waffle', .6, 1.82)):
            with patch.object(teleop, 'TURTLEBOT3_MODEL', model):
                for sign in (-1, 1):
                    self.assertEqual(teleop.check_linear_limit_velocity(sign * 10, .12), sign * .12)
                    self.assertEqual(teleop.check_angular_limit_velocity(sign * 10, .7), sign * .7)
                    self.assertEqual(teleop.check_linear_limit_velocity(sign * 10, 100), sign * max_linear)
                    self.assertEqual(teleop.check_angular_limit_velocity(sign * 10, 100), sign * max_angular)
                self.assertEqual(teleop.check_linear_limit_velocity(.1, 0), 0)

    def capture(self, stamp=1.8, now=2., gazebo_available=True):
        tf = TransformStamped()
        tf.header.stamp.sec = int(stamp)
        tf.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
        # A changed/rotated map origin must not trigger a historical-offset gate.
        tf.transform.translation.x = 8.
        tf.transform.translation.y = -3.
        buffer = Mock()
        buffer.lookup_transform.return_value = tf
        node = Mock()
        node.get_clock().now.return_value = SimpleNamespace(nanoseconds=round(now * 1e9))
        gazebo = Mock()
        position = Point(x=-1.75, y=-4.5)
        gazebo.robot_pose_sample.return_value = (position, tf.header.stamp) if gazebo_available else None
        with patch('waypoint_capture.rclpy.spin_once'):
            result = capture_waypoint_pose(node, buffer, gazebo, timeout_sec=.02)
        return result, buffer, position

    def test_map_and_gazebo_are_read_at_the_same_time_despite_different_origins(self):
        (pose, gazebo), buffer, position = self.capture()
        self.assertEqual(pose.translation.x, 8.)
        self.assertIs(gazebo, position)
        args = buffer.lookup_transform.call_args.args
        self.assertEqual(args[:2], ('map', 'base_footprint'))
        self.assertEqual(args[2].nanoseconds, 1_800_000_000)

    def test_map_only_capture_still_requires_fresh_tf(self):
        (_, position), buffer, _ = self.capture(gazebo_available=False)
        self.assertIsNone(position)
        self.assertEqual(buffer.lookup_transform.call_args.args[2].nanoseconds, 0)

    def test_stale_future_and_uninitialised_clock_samples_are_rejected(self):
        for stamp, now in ((1., 2.), (3., 2.), (0., 2.), (1., 0.)):
            with self.subTest(stamp=stamp, now=now), self.assertRaises(WaypointCaptureError):
                self.capture(stamp=stamp, now=now)


if __name__ == '__main__':
    unittest.main()
