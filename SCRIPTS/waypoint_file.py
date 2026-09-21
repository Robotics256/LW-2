"""Reading named Nav2 waypoints from the project's ``points.txt`` format.

Each non-empty row has the following fields::

    name, map_x, map_y, orientation_z, orientation_w[, gazebo_x, gazebo_y]

The optional Gazebo coordinates are metadata and are not used as a Nav2 goal.
"""

from pathlib import Path

from geometry_msgs.msg import Point, PoseStamped


def load_waypoints(file_path):
    """Return named poses from *file_path* or raise a clear error."""
    path = Path(file_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"Файл точек не найден: {path}")

    waypoints = []
    names = set()
    with path.open(encoding="utf-8") as points:
        for line_number, line in enumerate(points, 1):
            if not line.strip():
                continue
            parts = [part.strip() for part in line.split(",")]
            if len(parts) < 5 or not parts[0]:
                raise ValueError(
                    f"Строка {line_number} в {path}: ожидается "
                    "имя,x,y,qz,qw[,gazebo_x,gazebo_y]"
                )
            if parts[0] in names:
                raise ValueError(f"Строка {line_number} в {path}: имя '{parts[0]}' повторяется")
            try:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(parts[1])
                pose.pose.position.y = float(parts[2])
                pose.pose.orientation.z = float(parts[3])
                pose.pose.orientation.w = float(parts[4])
                waypoint = {"name": parts[0], "pose": pose}
                if len(parts) >= 7:
                    waypoint["gazebo_position"] = Point(
                        x=float(parts[5]), y=float(parts[6]), z=0.0
                    )
            except ValueError as error:
                raise ValueError(f"Строка {line_number} в {path} содержит нечисловую координату") from error
            names.add(parts[0])
            waypoints.append(waypoint)
    return waypoints


def waypoints_by_name(file_path):
    """Return a name-to-waypoint mapping for command-line clients."""
    return {waypoint["name"]: waypoint for waypoint in load_waypoints(file_path)}
