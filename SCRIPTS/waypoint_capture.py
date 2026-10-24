"""Read a fresh map pose, paired with Gazebo at the same simulation time."""

import time

import rclpy
from rclpy.time import Time
from tf2_ros import TransformException


class WaypointCaptureError(ValueError):
    pass


def capture_waypoint_pose(node, tf_buffer, gazebo, timeout_sec=5.0, max_age_sec=0.5):
    # Process /clock and queued TF after the blocking name prompt. Never use a
    # pose captured before the student finished entering the point name.
    drain_until = time.monotonic() + 0.2
    while time.monotonic() < drain_until:
        rclpy.spin_once(node, timeout_sec=0.02)
    sample = gazebo.robot_pose_sample()
    position, stamp = sample if sample is not None else (None, None)
    query_time = Time.from_msg(stamp) if stamp is not None else Time()
    if stamp is not None and query_time.nanoseconds <= 0:
        raise WaypointCaptureError('Gazebo ещё не публикует время симуляции. Снимите паузу.')

    deadline = time.monotonic() + timeout_sec
    last_error = 'Нет свежего преобразования map → base_footprint.'
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        try:
            transform = tf_buffer.lookup_transform('map', 'base_footprint', query_time)
        except TransformException as error:
            last_error = str(error)
            continue
        now = node.get_clock().now().nanoseconds
        recorded = Time.from_msg(transform.header.stamp).nanoseconds
        age = (now - recorded) / 1e9
        if now <= 0 or recorded <= 0 or not 0 <= age <= max_age_sec:
            last_error = f'Поза TF устарела или часы рассинхронизированы (возраст {age:.2f} с).'
            continue
        return transform.transform, position
    raise WaypointCaptureError(
        f'Точка не сохранена: {last_error} '
        'Проверьте /clock, TF, Cartographer/AMCL и use_sim_time.')
