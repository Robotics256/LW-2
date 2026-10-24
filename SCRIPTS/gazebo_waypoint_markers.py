"""Create visible Gazebo Classic entities for named saved waypoints."""

import os

import rclpy
from gazebo_msgs.srv import GetEntityState, SpawnEntity


# Static, visual-only marks do not affect lidar, Nav2, or physics.
FLOOR_MARK_SDF = '''<sdf version="1.6">
  <model name="waypoint_mark">
    <static>true</static>
    <link name="link">
      <!-- The disc clears the 2 mm floor paint by 1 mm. -->
      <pose>0 0 0.0204166666666667 0 0 0</pose>
      <visual name="green_floor_circle">
        <geometry><cylinder><radius>0.1833333333333333</radius><length>0.0333333333333333</length></cylinder></geometry>
        <material>
          <ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse>
          <emissive>0 0.35 0 1</emissive><specular>0 0.2 0 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''


class GazeboWaypointPublisher:
    """Spawn waypoint markers and query the robot through gazebo_ros services."""

    def __init__(self, node):
        self._node = node
        self._spawned_names = set()
        self._robot_name = os.environ.get('TURTLEBOT3_MODEL', 'burger')
        self._spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
        self._state_client = node.create_client(
            GetEntityState, '/gazebo/get_entity_state')

    def wait_until_ready(self, timeout_sec=30.0):
        """Wait until Gazebo Classic exposes its entity-spawn service."""
        return self._spawn_client.wait_for_service(timeout_sec=timeout_sec)

    def publish(self, waypoints, _tf_buffer=None):
        """Create models for waypoints that have not been spawned this session."""
        if not self._spawn_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warning(
                'Gazebo service /spawn_entity is not available; markers were skipped.')
            return

        for waypoint in waypoints:
            name = waypoint['name']
            if name in self._spawned_names:
                continue
            position = waypoint.get('gazebo_position')
            if position is None:
                self._node.get_logger().warning(
                    f"Gazebo marker '{name}' was skipped: its exact Gazebo "
                    'position is not stored.')
                continue

            request = SpawnEntity.Request()
            request.name = f'waypoint_{name}'
            request.xml = FLOOR_MARK_SDF
            request.reference_frame = 'world'
            request.initial_pose.position.x = position.x
            request.initial_pose.position.y = position.y
            request.initial_pose.orientation.w = 1.0
            future = self._spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            response = future.result() if future.done() else None
            if response is not None and response.success:
                self._spawned_names.add(name)
            elif response is not None and 'already exists' in response.status_message.lower():
                self._spawned_names.add(name)
            else:
                detail = response.status_message if response is not None else 'timeout'
                self._node.get_logger().warning(
                    f"Gazebo marker '{name}' was not created: {detail}")

    def robot_position(self):
        """Read the robot model position in the Gazebo world frame."""
        sample = self.robot_pose_sample()
        return sample[0] if sample is not None else None

    def robot_pose_sample(self):
        """Return world position and simulation timestamp from one service reply."""
        if not self._state_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warning(
                'Gazebo service /gazebo/get_entity_state is not available.')
            return None

        request = GetEntityState.Request()
        request.name = self._robot_name
        request.reference_frame = 'world'
        future = self._state_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        response = future.result() if future.done() else None
        if response is None or not response.success:
            self._node.get_logger().warning(
                f"Gazebo model '{self._robot_name}' was not found.")
            return None
        return response.state.pose.position, response.header.stamp
