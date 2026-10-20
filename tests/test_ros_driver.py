"""Opt-in integration with a real ROS action server, without Gazebo or MQTT.

source /opt/ros/humble/setup.bash
LW2_RUN_ROS_TESTS=1 ROS_DOMAIN_ID=182 python3 -m unittest discover -s tests -p test_ros_driver.py -v
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "SCRIPTS"))


@unittest.skipUnless(os.environ.get("LW2_RUN_ROS_TESTS") == "1", "opt-in ROS integration")
class RosDriverTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from nav2_msgs.action import NavigateToPose
        from nav2_msgs.srv import ClearEntireCostmap
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from mqtt_driver import MqttNavNode

        rclpy.init()
        self.rclpy = rclpy
        self.received = []
        self.cancel_enabled = True
        self.cancels = 0
        self.complete = threading.Event()
        self.server = rclpy.create_node("lw2_test_nav2")
        group = ReentrantCallbackGroup()

        def accept(goal):
            # Exercise Stop while the goal acknowledgement is still pending.
            time.sleep(.15)
            return GoalResponse.ACCEPT

        def cancel(goal):
            self.cancels += 1
            return CancelResponse.ACCEPT if self.cancel_enabled else CancelResponse.REJECT

        def execute(goal):
            self.received.append(goal.request)
            while rclpy.ok() and not self.complete.is_set():
                if goal.is_cancel_requested:
                    goal.canceled()
                    return NavigateToPose.Result()
                time.sleep(.01)
            goal.succeed()
            return NavigateToPose.Result()

        self.action = ActionServer(self.server, NavigateToPose, "navigate_to_pose",
                                   execute_callback=execute, goal_callback=accept,
                                   cancel_callback=cancel, callback_group=group)
        self.services = [self.server.create_service(
            ClearEntireCostmap, path, lambda request, response: response,
            callback_group=group) for path in (
                "local_costmap/clear_entirely_local_costmap",
                "global_costmap/clear_entirely_global_costmap")]
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.server)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.directory = tempfile.TemporaryDirectory()
        points = Path(self.directory.name) / "points.txt"
        points.write_text("parking,0,0,0,1\nshipping,1,0,0.7071068,0.7071068\n")
        self.paho_patch = patch("mqtt_driver.mqtt.Client")
        self.client = self.paho_patch.start().return_value
        self.responses = []
        self.client.publish.side_effect = lambda topic, payload, **kwargs: self.responses.append(json.loads(payload))
        self.driver = MqttNavNode(points_file=str(points), trail_color_number=False,
                                 cargo_config={"floor_markers_enabled": False,
                                               "robot_color_enabled": False})

    def tearDown(self):
        self.complete.set()
        self.driver.close()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.action.destroy()
        self.server.destroy_node()
        self.rclpy.shutdown()
        self.paho_patch.stop()
        self.directory.cleanup()

    def send(self, data):
        from mqtt_driver import MODULE_NAME
        payload = {"Module": {"Name": MODULE_NAME, "Type": "delivery", "UID": 123, "Data": data}}
        self.driver.on_message(None, None, SimpleNamespace(payload=json.dumps(payload).encode()))

    def pump_until(self, predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            self.rclpy.spin_once(self.driver, timeout_sec=.02)
        self.assertTrue(predicate(), f"Timed out; responses: {self.responses}")

    def test_stop_during_goal_ack_then_new_goal_and_cargo_chain(self):
        self.send({"Task": {"Id": 1, "Type": "Point", "Data": "parking"}})
        self.pump_until(lambda: self.driver.goal_send is not None)
        self.send({"Request": "status"})
        self.send({"Task": {"Id": 2, "Type": "Stop"}})
        self.pump_until(lambda: 2 in self.driver.tasks.history)
        self.assertEqual(self.driver.tasks.history[1]["Status"], 2)
        self.assertEqual(self.driver.tasks.history[2]["Status"], 0)
        self.assertGreaterEqual(self.cancels, 1)
        self.assertIn("position_only_navigation.xml", self.received[0].behavior_tree)
        self.assertTrue(any("StatusInfo" in item["Module"]["Data"] for item in self.responses))

        self.complete.set()
        self.send({"Tasks": [
            {"Id": 3, "Type": "Point", "Data": "shipping", "Mode": "orientation"},
            {"Id": 4, "Type": "Upload"},
            {"Id": 5, "Type": "Point", "Data": "parking", "Mode": "proximity"},
        ]})
        self.pump_until(lambda: 5 in self.driver.tasks.history)
        self.assertTrue(all(self.driver.tasks.history[identifier]["Status"] == 0 for identifier in (3, 4, 5)))
        self.assertIn("orientation_aware_navigation.xml", self.received[1].behavior_tree)
        self.assertIn("proximity_navigation.xml", self.received[2].behavior_tree)
        self.assertTrue(self.driver.tasks.loaded)
        self.assertEqual(self.responses[-1]["Module"]["Status"], 0)
        self.send({"Request": "history", "Limit": 2})
        self.pump_until(lambda: "History" in self.responses[-1]["Module"]["Data"])
        self.assertEqual([item["Id"] for item in self.responses[-1]["Module"]["Data"]["History"]["Tasks"]], [5, 4])

    def test_rejected_cancellation_blocks_new_goals_until_stop_is_retried(self):
        self.cancel_enabled = False
        self.send({"Task": {"Id": 11, "Type": "Point", "Data": "parking"}})
        self.pump_until(lambda: bool(self.received))
        self.send({"Task": {"Id": 12, "Type": "Stop"}})
        self.pump_until(lambda: 12 in self.driver.tasks.history)
        self.assertEqual(self.driver.tasks.history[12]["Status"], 2)
        self.assertTrue(self.driver.tasks.blocked)
        self.send({"Task": {"Id": 13, "Type": "Point", "Data": "shipping"}})
        self.pump_until(lambda: any(item["Module"]["Data"].get("Task", {}).get("Id") == 13 for item in self.responses))
        self.assertEqual(len(self.received), 1)
        self.cancel_enabled = True
        self.send({"Task": {"Id": 14, "Type": "Stop"}})
        self.pump_until(lambda: 14 in self.driver.tasks.history)
        self.assertEqual(self.driver.tasks.history[14]["Status"], 0)
        self.assertFalse(self.driver.tasks.blocked)


if __name__ == "__main__":
    unittest.main()
