"""RViz markers for waypoints stored in ``points.txt``."""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


WAYPOINT_MARKERS_TOPIC = '/waypoints'
# The reduced cylinder rests 3 mm above the floor paint.  This keeps it visible
# over parking/loading labels without making the waypoint appear to float.
# 25% larger than the previous 16 cm disc.
MARKER_DIAMETER = 0.20
MARKER_THICKNESS = 0.04 * 2 / 3 * 1.25
MARKER_HEIGHT = .003 + MARKER_THICKNESS / 2


class WaypointMarkerPublisher:
    """Publish saved waypoints as green circles resting on the map floor."""

    def __init__(self, node):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._node = node
        self._publisher = node.create_publisher(MarkerArray, WAYPOINT_MARKERS_TOPIC, qos)

    def publish(self, waypoints):
        """Publish ``{'name': str, 'pose': PoseStamped}`` waypoint dictionaries."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        name_occurrences = {}

        for waypoint in waypoints:
            pose = waypoint['pose'].pose
            marker = Marker()
            marker.header.frame_id = 'map'
            # Saved map coordinates are timeless, including after a /clock reset.
            # RViz groups markers by ``ns`` in its display tree.  Using the
            # saved name here makes a point named, for example, "loading",
            # appear as "loading" rather than ``saved_waypoints/3``.
            marker.ns = waypoint['name']
            marker.id = name_occurrences.get(marker.ns, 0)
            name_occurrences[marker.ns] = marker.id + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = pose.position.x
            marker.pose.position.y = pose.position.y
            # Keep the marker just over the map plane; otherwise it can be
            # hidden by the map texture and its zone labels in a top-down view.
            marker.pose.position.z = MARKER_HEIGHT
            marker.pose.orientation.w = 1.0
            marker.scale.x = MARKER_DIAMETER
            marker.scale.y = MARKER_DIAMETER
            marker.scale.z = MARKER_THICKNESS
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            markers.markers.append(marker)

        self._publisher.publish(markers)
