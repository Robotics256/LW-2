"""Draw a visual-only dotted robot trail in Gazebo Classic."""

import math
import os

from gazebo_msgs.srv import GetEntityState, SpawnEntity
from lw2_config import DEFAULT_CONFIG_FILE, load_trail_color as load_configured_trail_color


TRAIL_DOT_SDF = '''<sdf version="1.6">
  <model name="navigation_trail_dot">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.012 0 0 0</pose>
      <visual name="blue_dot">
        <geometry>
          <cylinder><radius>0.055</radius><length>0.012</length></cylinder>
        </geometry>
        <material>
          <ambient>0.05 0.25 1 1</ambient>
          <diffuse>0.05 0.25 1 1</diffuse>
          <emissive>0.02 0.08 0.4 1</emissive>
          <specular>0.1 0.1 0.3 1</specular>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>'''
DEFAULT_TRAIL_COLOR = (0.05, 0.25, 1.0)


def load_trail_color(color_number, config_file=DEFAULT_CONFIG_FILE):
    """Return the configured RGB colour for a trail number from 1 through 21."""
    if color_number is None:
        return DEFAULT_TRAIL_COLOR
    return load_configured_trail_color(color_number, config_file)


def trail_dot_sdf(color):
    """Create the visual-only Gazebo model XML for one coloured trail dot."""
    if color == DEFAULT_TRAIL_COLOR:
        return TRAIL_DOT_SDF
    red, green, blue = color
    ambient = f"{red} {green} {blue} 1"
    emissive = f"{red * 0.32} {green * 0.32} {blue * 0.32} 1"
    return (
        TRAIL_DOT_SDF.replace("blue_dot", "trail_dot")
        .replace("0.05 0.25 1 1", ambient)
        .replace("0.02 0.08 0.4 1", emissive)
    )



class GazeboTrail:
    """Sample the robot world pose and spawn non-colliding floor dots."""

    def __init__(
        self,
        node,
        spacing=0.18,
        color_number=None,
        config_file=DEFAULT_CONFIG_FILE,
    ):
        self._node = node
        self._spacing = spacing
        self._dot_sdf = trail_dot_sdf(
            load_trail_color(color_number, config_file)
        )
        self._robot_name = os.environ.get("TURTLEBOT3_MODEL", "burger")
        self._state_client = node.create_client(
            GetEntityState, "/gazebo/get_entity_state"
        )
        self._spawn_client = node.create_client(SpawnEntity, "/spawn_entity")
        self._active = False
        self._generation = 0
        self._trail_number = 0
        self._dot_number = 0
        self._last_position = None
        self._state_future = None
        self._spawn_futures = set()
        self._warned_state_failure = False

    def start(self, destination_name):
        """Begin a new trail at the robot's current Gazebo position."""
        if not self._state_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warning(
                "След отключён: сервис /gazebo/get_entity_state недоступен."
            )
            return False
        if not self._spawn_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warning(
                "След отключён: сервис /spawn_entity недоступен."
            )
            return False

        self._active = True
        self._generation += 1
        self._trail_number += 1
        self._dot_number = 0
        self._last_position = None
        self._state_future = None
        self._warned_state_failure = False
        self._node.get_logger().info(
            f"Запись пунктирного следа до точки '{destination_name}'"
        )
        self.sample()
        return True

    def stop(self):
        """Stop adding dots while leaving the completed trail in Gazebo."""
        self._active = False
        self._generation += 1
        self._state_future = None

    def sample(self):
        """Request one robot pose; the service response is handled asynchronously."""
        if not self._active:
            return
        if self._state_future is not None and not self._state_future.done():
            return

        request = GetEntityState.Request()
        request.name = self._robot_name
        request.reference_frame = "world"
        generation = self._generation
        future = self._state_client.call_async(request)
        self._state_future = future
        future.add_done_callback(
            lambda completed, current_generation=generation: self._on_state(
                completed, current_generation
            )
        )

    def _on_state(self, future, generation):
        if generation != self._generation or not self._active:
            return
        self._state_future = None

        try:
            response = future.result()
        except Exception as error:
            self._warn_state_failure(str(error))
            return
        if response is None or not response.success:
            detail = response.status_message if response is not None else "нет ответа"
            self._warn_state_failure(detail)
            return

        position = response.state.pose.position
        point = (position.x, position.y)
        if self._last_position is not None:
            distance = math.hypot(
                point[0] - self._last_position[0],
                point[1] - self._last_position[1],
            )
            if distance < self._spacing:
                return

        self._last_position = point
        self._spawn_dot(*point)

    def _spawn_dot(self, x, y):
        self._dot_number += 1
        name = (
            f"navigation_trail_{os.getpid()}_"
            f"{self._trail_number}_{self._dot_number}"
        )
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self._dot_sdf
        request.reference_frame = "world"
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.orientation.w = 1.0
        future = self._spawn_client.call_async(request)
        self._spawn_futures.add(future)
        future.add_done_callback(
            lambda completed, entity_name=name: self._on_spawn(
                completed, entity_name
            )
        )

    def _on_spawn(self, future, entity_name):
        self._spawn_futures.discard(future)
        try:
            response = future.result()
        except Exception as error:
            self._node.get_logger().warning(
                f"Точка следа '{entity_name}' не создана: {error}"
            )
            return
        if response is not None and not response.success:
            self._node.get_logger().warning(
                f"Точка следа '{entity_name}' не создана: "
                f"{response.status_message}"
            )

    def _warn_state_failure(self, detail):
        if self._warned_state_failure:
            return
        self._warned_state_failure = True
        self._node.get_logger().warning(
            f"Не удалось получить положение робота для следа: {detail}"
        )
