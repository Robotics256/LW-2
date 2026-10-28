"""Persistent load/unload floor marks and a loaded-state robot overlay."""

import os
import uuid
from copy import deepcopy

from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity


# Existing route dots end at z=0.018 and waypoint discs at about z=0.037.
# These 18 mm high marks start at z=0.045, so they remain visible above both.
TRIANGLE_SDF = '''<sdf version="1.6">
  <model name="cargo_floor_mark">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.045 0 0 0</pose>
      <visual name="cargo_triangle">
        <geometry>
          <polyline>
            <height>0.018</height>
            <point>-0.14 -0.11</point>
            <point>0.14 -0.11</point>
            <point>0.0 0.15</point>
          </polyline>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <emissive>{emissive}</emissive>
          <specular>0.15 0.15 0.15 1</specular>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>'''

LOADED_OVERLAY_SDF = '''<sdf version="1.6">
  <model name="loaded_robot_overlay">
    <static>true</static>
    <link name="link">
      <visual name="red_loaded_top">
        <geometry><cylinder><radius>0.105</radius><length>0.014</length></cylinder></geometry>
        <material>
          <ambient>1 0.02 0.02 1</ambient>
          <diffuse>1 0.02 0.02 1</diffuse>
          <emissive>0.35 0 0 1</emissive>
          <specular>0.25 0.1 0.1 1</specular>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>'''

MARK_COLORS = {
    "load": ("1 0.02 0.02 1", "0.35 0 0 1"),
    "unload": ("0.02 0.15 1 1", "0 0.05 0.35 1"),
}


class GazeboCargoState:
    """Apply visual cargo state without adding collisions or changing physics."""

    def __init__(self, node, floor_markers_enabled=True, robot_color_enabled=True):
        self._node = node
        self._floor_markers_enabled = floor_markers_enabled
        self._robot_color_enabled = robot_color_enabled
        self._robot_name = os.environ.get("TURTLEBOT3_MODEL", "burger")
        # Stable across driver restarts so an existing overlay can be reused
        # by Load and removed by Unload instead of becoming orphaned.
        self._overlay_name = f"{self._robot_name}_loaded_overlay"
        self._overlay_height = 0.145
        self._loaded = False
        self._follow_state_future = None
        self._follow_set_futures = set()

        self._state_client = node.create_client(
            GetEntityState, "/gazebo/get_entity_state"
        )
        self._spawn_client = node.create_client(SpawnEntity, "/spawn_entity")
        self._delete_client = node.create_client(DeleteEntity, "/delete_entity")
        self._set_state_client = node.create_client(
            SetEntityState, "/gazebo/set_entity_state"
        )

    @property
    def enabled(self):
        return self._floor_markers_enabled or self._robot_color_enabled

    @property
    def loaded(self):
        return self._loaded

    def apply(self, action, completion):
        """Apply ``load`` or ``unload`` and invoke completion(success, detail)."""
        if action not in MARK_COLORS:
            completion(False, f"unknown cargo action '{action}'")
            return
        if not self.enabled:
            self._loaded = action == "load"
            completion(True, "visual effects disabled by config")
            return

        required = [self._state_client]
        if self._floor_markers_enabled or (action == "load" and self._robot_color_enabled):
            required.append(self._spawn_client)
        if action == "unload" and self._robot_color_enabled:
            required.append(self._delete_client)
        if not all(client.wait_for_service(timeout_sec=0.25) for client in required):
            completion(False, "required Gazebo service is unavailable")
            return

        request = GetEntityState.Request()
        request.name = self._robot_name
        request.reference_frame = "world"
        future = self._state_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._on_action_pose(completed, action, completion)
        )

    def update(self):
        """Keep the red upper overlay aligned with a moving loaded robot."""
        if not (self._robot_color_enabled and self._loaded):
            return
        if self._follow_state_future is not None and not self._follow_state_future.done():
            return
        if not self._state_client.service_is_ready():
            return

        request = GetEntityState.Request()
        request.name = self._robot_name
        request.reference_frame = "world"
        future = self._state_client.call_async(request)
        self._follow_state_future = future
        future.add_done_callback(self._on_follow_pose)

    def _on_action_pose(self, future, action, completion):
        try:
            response = future.result()
        except Exception as error:
            completion(False, f"robot pose request failed: {error}")
            return
        if response is None or not response.success:
            detail = response.status_message if response is not None else "no response"
            completion(False, f"robot pose is unavailable: {detail}")
            return

        pose = response.state.pose
        operations = []
        if self._floor_markers_enabled:
            operations.append(("floor marker", self._spawn_marker(action, pose)))
        if self._robot_color_enabled:
            if action == "load":
                operations.append(("robot colour", self._spawn_overlay(pose)))
            else:
                operations.append(("robot colour", self._delete_overlay()))

        if not operations:
            self._loaded = action == "load"
            completion(True, "completed")
            return

        remaining = {"count": len(operations), "errors": []}
        for label, operation in operations:
            operation.add_done_callback(
                lambda done, current_label=label: self._finish_operation(
                    done, current_label, action, remaining, completion
                )
            )

    def _spawn_marker(self, action, pose):
        color, emissive = MARK_COLORS[action]
        request = SpawnEntity.Request()
        request.name = f"cargo_{action}_{uuid.uuid4().hex[:12]}"
        request.xml = TRIANGLE_SDF.format(color=color, emissive=emissive)
        request.reference_frame = "world"
        request.initial_pose = deepcopy(pose)
        request.initial_pose.position.z = 0.0
        return self._spawn_client.call_async(request)

    def _spawn_overlay(self, pose):
        request = SpawnEntity.Request()
        request.name = self._overlay_name
        request.xml = LOADED_OVERLAY_SDF
        request.reference_frame = "world"
        request.initial_pose = deepcopy(pose)
        request.initial_pose.position.z += self._overlay_height
        return self._spawn_client.call_async(request)

    def _delete_overlay(self):
        request = DeleteEntity.Request()
        request.name = self._overlay_name
        return self._delete_client.call_async(request)

    def _finish_operation(self, future, label, action, remaining, completion):
        try:
            response = future.result()
            success = response is not None and response.success
            detail = response.status_message if response is not None else "no response"
        except Exception as error:
            success = False
            detail = str(error)

        # Repeated load/unload commands are idempotent for the colour overlay.
        normalized = detail.lower()
        if label == "robot colour" and (
            "already exists" in normalized or "does not exist" in normalized
        ):
            success = True
        if label == "robot colour" and success:
            # Keep the moving overlay state correct even if the independent
            # floor-marker operation fails.
            self._loaded = action == "load"
        if not success:
            remaining["errors"].append(f"{label}: {detail}")

        remaining["count"] -= 1
        if remaining["count"] != 0:
            return
        if not remaining["errors"]:
            self._loaded = action == "load"
            completion(True, "completed")
        else:
            completion(False, "; ".join(remaining["errors"]))

    def _on_follow_pose(self, future):
        self._follow_state_future = None
        if not self._loaded or not self._set_state_client.service_is_ready():
            return
        try:
            response = future.result()
        except Exception:
            return
        if response is None or not response.success:
            return

        request = SetEntityState.Request()
        request.state.name = self._overlay_name
        request.state.reference_frame = "world"
        request.state.pose = deepcopy(response.state.pose)
        request.state.pose.position.z += self._overlay_height
        set_future = self._set_state_client.call_async(request)
        self._follow_set_futures.add(set_future)
        set_future.add_done_callback(self._follow_set_futures.discard)
