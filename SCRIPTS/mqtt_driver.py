#!/usr/bin/env python3
"""LW-2 JSON MQTT endpoint: navigation, cargo, status, history and Stop."""

import argparse
from copy import deepcopy
import json
import os
import queue
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType

from gazebo_cargo_state import GazeboCargoState
from gazebo_trail import GazeboTrail
from lw2_config import DEFAULT_CONFIG_FILE, load_cargo_config
from lw2_config import (WORKSPACE, MODULE_NAME, MODULE_TYPE, MQTT_BROKER, MQTT_PORT,
                        MQTT_COMMAND_TOPIC, MQTT_STATUS_TOPIC, configure_mqtt_auth)
from mqtt_tasks import JsonTasks
from navigation_modes import add_mode_arguments, add_trail_arguments, behavior_tree, validate_trail
from navigation_stop import NavigationStop
from waypoint_file import load_waypoints
from waypoint_markers import WaypointMarkerPublisher


WAYPOINTS_FILE = os.environ.get("TB3_POINTS_FILE", os.path.expanduser("~/points.txt"))
MQTT_SUBSCRIBE_TOPIC = MQTT_COMMAND_TOPIC
MQTT_PUBLISH_TOPIC = MQTT_STATUS_TOPIC


class MqttNavNode(Node):
    def __init__(self, navigation_mode="position", trail_color_number=None,
                 cargo_config=None, config_file=DEFAULT_CONFIG_FILE,
                 points_file=WAYPOINTS_FILE, history_size=100, max_tasks=100):
        super().__init__("mqtt_nav_node", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.navigator = BasicNavigator()
        self.navigator.set_parameters([Parameter("use_sim_time", value=True)])
        self.waypoints = {point["name"]: point for point in load_waypoints(points_file)}
        self.markers = WaypointMarkerPublisher(self)
        self.markers.publish(list(self.waypoints.values()))
        self.trail = (GazeboTrail(self, color_number=trail_color_number, config_file=config_file)
                      if trail_color_number is not False else None)
        self.cargo_visualizer = GazeboCargoState(self, **(cargo_config or {}))
        self.velocity = self.navigator.create_publisher(Twist, "cmd_vel", 10)
        self.stopper = NavigationStop(self.navigator, self.velocity)
        self.messages = queue.Queue()
        self.tasks = JsonTasks(self.publish_data, navigation_mode, history_size, max_tasks)
        self.goal_task = None
        self.goal_send = None
        self.goal_deadline = None
        self.goal_send_timed_out = False
        self.cargo_task = None

        self.mqtt_client = mqtt.Client(client_id=f"mqtt-nav-{uuid.uuid4().hex[:12]}")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        # Optional credentials work for all LW-2 senders as well.
        configure_mqtt_auth(self.mqtt_client)
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.mqtt_client.connect_async(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()
        # Requests and Stop must keep working while Gazebo is paused or before
        # its first /clock message. Goal stamps still use simulation time.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.navigation_loop, clock=self.wall_clock)
        self.get_logger().info(
            f"JSON MQTT: {len(self.waypoints)} точек; Mode={navigation_mode}; "
            f"след={'вкл' if self.trail else 'выкл'}. Перед движением задайте начальную позу AMCL.")

    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f"MQTT connection failed: {rc}")
            return
        client.subscribe(MQTT_SUBSCRIBE_TOPIC, qos=1)
        self.get_logger().info(f"MQTT: подписан на {MQTT_SUBSCRIBE_TOPIC}")

    def on_message(self, client, userdata, message):
        # Nav2 and task state are touched only by the ROS timer, never by Paho.
        try:
            module = json.loads(message.payload.decode("utf-8"))["Module"]
            if not isinstance(module, dict):
                raise ValueError("Module must be an object")
            if module.get("Name") not in (MODULE_NAME, "all"):
                return
            if module.get("Type", module.get("ModuleType")) != MODULE_TYPE:
                return
            self.messages.put(module)
        except (ValueError, KeyError, TypeError, UnicodeDecodeError) as error:
            self.get_logger().warning(f"Некорректный JSON MQTT: {error}")

    def publish_data(self, data):
        payload = {"Module": {"Name": MODULE_NAME, "Type": MODULE_TYPE,
                              "UID": uuid.uuid4().int & ((1 << 63) - 1),
                              "Time": int(time.time() * 1000),
                              "Status": 2 if self.tasks.blocked else int(self.tasks.busy),
                              "Data": data}}
        self.mqtt_client.publish(MQTT_PUBLISH_TOPIC, json.dumps(payload), qos=1, retain=False)

    def navigation_loop(self):
        self.tasks.loaded = self.cargo_visualizer.loaded
        for _ in range(50):
            try:
                module = self.messages.get_nowait()
            except queue.Empty:
                break
            self.tasks.handle(module.get("Data"), module.get("UID"))
        if self.tasks.stop_task:
            self._stop_navigation()
            return

        self.cargo_visualizer.update()
        rclpy.spin_once(self.navigator, timeout_sec=0)
        self._tick_navigation()
        if self.tasks.blocked:
            self.stopper.brake()
            return
        if self.cargo_task is None:
            cargo = self.tasks.take_cargo()
            if cargo is not None:
                self._start_cargo(cargo)
        pending = self.tasks.tasks.get(self.tasks.head)
        if pending and pending["Type"] != "Point" and self.cargo_task is not None:
            return
        task = self.tasks.take_main()
        if task is not None:
            if task["Type"] == "Point":
                self._start_point(task)
            else:
                self._start_cargo(task)

    def _start_point(self, task):
        waypoint = self.waypoints.get(task["Data"])
        if waypoint is None:
            self.tasks.finish(task["Id"], False, f"point '{task['Data']}' not exists")
            return
        self.goal_task = task
        self.goal_deadline = time.monotonic() + 5.0
        self.goal_send_timed_out = False

    def _tick_navigation(self):
        if self.goal_task is None:
            return
        if self.tasks.blocked and self.goal_send is None and self.navigator.result_future is None:
            return
        if self.goal_send is None and self.navigator.result_future is None:
            if self.navigator.nav_to_pose_client.server_is_ready():
                goal = NavigateToPose.Goal()
                goal.pose = deepcopy(self.waypoints[self.goal_task["Data"]]["pose"])
                goal.pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal.behavior_tree = behavior_tree(self.goal_task["Mode"])
                self.goal_send = self.navigator.nav_to_pose_client.send_goal_async(
                    goal, self.navigator._feedbackCallback)
                self.goal_deadline = time.monotonic() + 5.0
            elif time.monotonic() >= self.goal_deadline:
                self._finish_point(False, "NavigateToPose server unavailable (5 s)")
            return
        if self.goal_send is not None:
            if not self.goal_send.done():
                if time.monotonic() >= self.goal_deadline and not self.goal_send_timed_out:
                    self.goal_send_timed_out = True
                    self.tasks.blocked = True
                    self.tasks.finish(self.goal_task["Id"], False,
                                      "Nav2 goal acknowledgement timed out; send Stop")
                return
            if not self._resolve_goal():
                return
        result = self.navigator.result_future
        if result is not None and result.done():
            try:
                status = result.result().status
                self._finish_point(status == GoalStatus.STATUS_SUCCEEDED,
                                   "task completed" if status == GoalStatus.STATUS_SUCCEEDED
                                   else f"navigation result: {status}")
            except Exception as error:
                self.tasks.blocked = True
                self._finish_point(False, f"Nav2 result failed: {error}; send Stop", reset=False)
        elif self.trail:
            self.trail.sample()

    def _resolve_goal(self):
        try:
            handle = self.goal_send.result()
            self.goal_send = None
            if not handle.accepted:
                self._finish_point(False, "Nav2 rejected the goal")
                return False
            self.navigator.goal_handle = handle
            self.navigator.result_future = handle.get_result_async()
            if self.trail and not self.goal_send_timed_out and not self.tasks.stop_task:
                self.trail.start(self.goal_task["Data"])
            return True
        except Exception as error:
            self.tasks.blocked = True
            self._finish_point(False, f"Nav2 goal failed: {error}; send Stop", reset=False)
            return False

    def _finish_point(self, success, report, reset=True):
        if self.trail:
            self.trail.stop()
        if self.goal_task is not None:
            self.tasks.finish(self.goal_task["Id"], success, report)
        if reset:
            self.goal_task = None
            self.goal_send = None
            self.navigator.goal_handle = None
            self.navigator.result_future = None

    def _start_cargo(self, task):
        identifier = task["Id"]
        self.cargo_task = task
        action = "load" if task["Type"] == "Upload" else "unload"

        def completed(success, detail):
            self.tasks.loaded = self.cargo_visualizer.loaded
            if self.tasks.tasks.get(identifier) is task:
                self.tasks.finish(identifier, success,
                                  f"{action} completed" if success else f"{action} failed: {detail}")
            if self.cargo_task is task:
                self.cargo_task = None

        self.cargo_visualizer.apply(action, completed)

    def _stop_navigation(self):
        try:
            # Stop can arrive while Nav2 is still acknowledging a goal. Keep
            # that future until its handle is known so a late goal cannot move.
            if self.goal_send is not None:
                if not self.stopper.wait(self.goal_send, 5.0):
                    raise RuntimeError("Nav2 goal acknowledgement timed out")
                if not self._resolve_goal() and self.goal_task is not None:
                    raise RuntimeError("Nav2 goal acknowledgement failed")
            self.stopper.stop()
        except Exception as error:
            self.tasks.finish_stop(False, f"Stop failed: {error}; retry with a new Id")
            self.get_logger().error(str(error))
            return
        if self.trail:
            self.trail.stop()
        self.goal_task = None
        self.goal_send = None
        self.cargo_task = None
        self.tasks.finish_stop(True, "robot stopped")

    def close(self):
        try:
            if self.goal_task is not None:
                self._stop_navigation()
        finally:
            self.stopper.brake()
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
            self.navigator.destroy_node()
            self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="LW-2 JSON MQTT → Nav2 driver")
    add_mode_arguments(parser)
    add_trail_arguments(parser)
    parser.add_argument("--points-file", default=WAYPOINTS_FILE)
    parser.add_argument("--config", "--cargo-config", default=str(DEFAULT_CONFIG_FILE))
    parser.add_argument("--task-history", type=int, default=100)
    parser.add_argument("--max-tasks", type=int, default=100)
    arguments = parser.parse_args()
    validate_trail(parser, arguments.trail)
    if arguments.task_history < 1 or arguments.max_tasks < 1:
        parser.error("--task-history и --max-tasks должны быть положительными")
    try:
        cargo_config = load_cargo_config(arguments.config)
    except ValueError as error:
        parser.error(str(error))
    rclpy.init()
    node = None
    try:
        node = MqttNavNode(arguments.navigation_mode, arguments.trail, cargo_config,
                           arguments.config, arguments.points_file,
                           arguments.task_history, arguments.max_tasks)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
