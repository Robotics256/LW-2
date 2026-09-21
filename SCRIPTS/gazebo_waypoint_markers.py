"""Create visible Gazebo Sim entities for named saved waypoints."""

import os
import re
import subprocess

from geometry_msgs.msg import Point


# The model is deliberately static and has no collision geometry: it is a
# visual mark floating over the floor labels, not an obstacle for Nav2 or the
# physics engine.
FLOOR_MARK_SDF = '''<sdf version="1.9">
  <model name="waypoint_mark">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.12 0 0 0</pose>
      <visual name="green_floor_circle">
        <geometry><cylinder><radius>0.22</radius><length>0.04</length></cylinder></geometry>
        <material>
          <ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse>
          <emissive>0 0.35 0 1</emissive>
          <specular>0 0.2 0 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''


class GazeboWaypointPublisher:
    """Spawn one green floor circle per waypoint in the running Gazebo world."""

    def __init__(self, node):
        self._node = node
        self._spawned_names = set()
        self._robot_name = os.environ.get('TURTLEBOT3_MODEL', 'waffle')

    def publish(self, waypoints, tf_buffer):
        """Create models for waypoints that have not been spawned this session."""
        for waypoint in waypoints:
            name = waypoint['name']
            if name in self._spawned_names:
                continue

            position = self._gazebo_position(waypoint)
            if position is None:
                continue
            command = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', name,
                '-string', FLOOR_MARK_SDF,
                '-x', str(position.x), '-y', str(position.y), '-z', '0.0',
            ]
            try:
                # Gazebo's create utility discovers the one active world when
                # no -world is given. It must stay non-blocking so teleop is
                # immediately usable after a point is recorded.
                subprocess.Popen(
                    command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=os.environ.copy(),
                )
                self._spawned_names.add(name)
            except OSError as error:
                self._node.get_logger().error(
                    f"Unable to start Gazebo marker '{name}': {error}")

    def _gazebo_position(self, waypoint):
        """Use only coordinates reported by Gazebo itself."""
        # A newly saved point includes the robot's actual Gazebo world pose.
        if 'gazebo_position' in waypoint:
            return waypoint['gazebo_position']
        self._node.get_logger().warning(
            f"Gazebo marker '{waypoint['name']}' was skipped: its exact "
            'Gazebo position is not stored.')
        return None

    def robot_position(self):
        """Read the robot's actual position from Gazebo Transport."""
        environment = os.environ.copy()
        world = environment.get('TB3_GAZEBO_WORLD', 'warehouse')
        try:
            output = subprocess.run(
                ['gz', 'topic', '-e', '-n', '1',
                 '-t', f'/world/{world}/pose/info'],
                env=environment, text=True, capture_output=True,
                timeout=8, check=True).stdout
        except (OSError, subprocess.SubprocessError) as error:
            self._node.get_logger().warning(f'Cannot read Gazebo robot pose: {error}')
            return None

        # Each entry in Pose_V is a protobuf block. Select the block named for
        # the robot and read its world X/Y values, not its ROS odometry.
        entry = re.search(
            rf'pose \{{\s+name: "{re.escape(self._robot_name)}".*?\n\}}\npose \{{',
            output + '\npose {', re.DOTALL)
        if entry is None:
            self._node.get_logger().warning(
                f"Gazebo model '{self._robot_name}' was not found in pose data.")
            return None
        values = entry.group(0)
        x_match = re.search(r'position \{\s+x: ([^\s]+)', values)
        y_match = re.search(r'\n\s*y: ([^\s]+)', values)
        if x_match is None or y_match is None:
            return None
        return Point(x=float(x_match.group(1)), y=float(y_match.group(1)), z=0.0)
