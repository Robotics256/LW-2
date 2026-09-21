#!/usr/bin/env python3

import argparse
import os
import select
import subprocess
import sys
import time


def bootstrap_ros_environment():
    """Allow the teleop script to be run directly from a new terminal."""
    if os.environ.get('_TB3_TELEOP_ENV_READY') == '1':
        return

    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ros_setup = '/opt/ros/jazzy/setup.bash'
    workspace_setup = os.path.join(project_dir, 'ros2_ws', 'install', 'setup.bash')
    if not os.path.isfile(ros_setup) or not os.path.isfile(workspace_setup):
        print(
            'ROS 2 Jazzy или workspace не найдены. Сначала выполните '
            './LW-2/SCRIPTS/S1_installRos2.sh.',
            file=sys.stderr,
        )
        raise SystemExit(1)

    environment = os.environ.copy()
    # S2_startSim.sh records the private Gazebo Transport partition. This
    # lets teleop, normally run in a second terminal, create entities in the
    # same Gazebo instance.
    partition_file = os.path.join(
        '/tmp', f"tb3_warehouse_{environment.get('USER', 'user')}.gz_partition")
    if not environment.get('GZ_PARTITION') and os.path.isfile(partition_file):
        with open(partition_file, encoding='utf-8') as file:
            partition = file.read().strip()
        if partition:
            environment['GZ_PARTITION'] = partition
    environment['_TB3_TELEOP_ENV_READY'] = '1'
    environment.setdefault('TURTLEBOT3_MODEL', 'waffle')
    command = 'source "$1" && source "$2" && exec python3 "$3" "${@:4}"'
    os.execvpe(
        '/bin/bash',
        [
            '/bin/bash', '-c', command, 'teleop-bootstrap', ros_setup,
            workspace_setup, os.path.abspath(__file__), *sys.argv[1:],
        ],
        environment,
    )


bootstrap_ros_environment()

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from gazebo_waypoint_markers import GazeboWaypointPublisher
from waypoint_markers import WaypointMarkerPublisher

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.6
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.6)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop
g : stop end save point in points file

Russian layout: ц/ч, ф/в, ы, п work as w/x, a/d, s, g.

CTRL-C to quit
"""

e = """
Communications Failed
"""

TELEOP_CMD_TOPIC = 'cmd_vel_teleop'

# Physical positions of the teleop keys on a Russian keyboard layout.
RUSSIAN_TELEOP_KEYS = str.maketrans({
    'ц': 'w', 'ф': 'a', 'ы': 's', 'в': 'd', 'ч': 'x', 'п': 'g',
    'Ц': 'w', 'Ф': 'a', 'Ы': 's', 'В': 'd', 'Ч': 'x', 'П': 'g',
})


def start_collision_monitor():
    """Start a private Nav2 safety filter for this teleop session.

    The monitor uses a private input topic, so a running Nav2 instance keeps
    its own collision monitor and command chain. If either safety process
    exits, teleop still writes only to the private topic and the robot stops.
    """
    config = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'teleop_collision_monitor.yaml')
    if not os.path.isfile(config):
        raise RuntimeError(f'Collision Monitor config is missing: {config}')

    monitor_executable = os.path.join(
        get_package_prefix('nav2_collision_monitor'),
        'lib', 'nav2_collision_monitor', 'collision_monitor')
    manager_executable = os.path.join(
        get_package_prefix('nav2_lifecycle_manager'),
        'lib', 'nav2_lifecycle_manager', 'lifecycle_manager')
    if not os.path.isfile(monitor_executable) or not os.path.isfile(manager_executable):
        raise RuntimeError('nav2_collision_monitor is not installed.')

    # Start the actual binaries, rather than the `ros2 run` wrapper, so Ctrl+C
    # reliably terminates both nodes and cannot leave background duplicates.
    monitor = subprocess.Popen([
        monitor_executable,
        '--ros-args', '-r', '__node:=teleop_collision_monitor',
        '--params-file', config, '-p', 'use_sim_time:=true',
    ])
    # Give the lifecycle node time to appear before activation is requested.
    time.sleep(0.5)
    manager = subprocess.Popen([
        manager_executable,
        '--ros-args', '-r', '__node:=teleop_collision_manager',
        '-p', "node_names:=['teleop_collision_monitor']",
        '-p', 'autostart:=true', '-p', 'use_sim_time:=true',
    ])
    time.sleep(0.5)
    if monitor.poll() is not None or manager.poll() is not None:
        for process in (manager, monitor):
            if process.poll() is None:
                process.terminate()
        raise RuntimeError('Collision Monitor did not start; robot commands remain blocked.')
    print('Collision Monitor: ON (/cmd_vel_teleop -> /cmd_vel, slowdown + stop zones).')
    return monitor, manager


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


def robot_pose(tf_buffer, node, frame):
    """Return the latest ``frame -> base_footprint`` transform, if available."""
    # Gazebo may publish the first map transform a few seconds after Nav2 or
    # Cartographer starts. Keep G reliable without forcing a second key press.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            return tf_buffer.lookup_transform(
                frame, 'base_footprint', Time(), timeout=Duration(seconds=0.2)).transform
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.05)
    return None


def load_waypoints(file_path):
    """Read the named waypoint format used by the teleop and MQTT driver."""
    from geometry_msgs.msg import Point, PoseStamped

    waypoints = []
    if not os.path.isfile(file_path):
        return waypoints
    with open(file_path, encoding='utf-8') as points:
        for line_number, line in enumerate(points, 1):
            parts = [part.strip() for part in line.split(',')]
            if len(parts) < 5 or not parts[0]:
                continue
            try:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(parts[1])
                pose.pose.position.y = float(parts[2])
                pose.pose.orientation.z = float(parts[3])
                pose.pose.orientation.w = float(parts[4])
                waypoint = {'name': parts[0], 'pose': pose}
                # Optional Gazebo world coordinates preserve the exact
                # simulated position even when Cartographer changes map->odom.
                if len(parts) >= 7:
                    waypoint['gazebo_position'] = Point(
                        x=float(parts[5]), y=float(parts[6]), z=0.0)
                waypoints.append(waypoint)
            except ValueError:
                print(f"Skipping invalid waypoint line {line_number}.")
    return waypoints


def main(points_file, collision_monitor=False):
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    safety_monitor = None
    safety_manager = None
    if collision_monitor:
        safety_monitor, safety_manager = start_collision_monitor()

    rclpy.init()
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    qos = QoSProfile(depth=10)
    node = rclpy.create_node(
        'teleop_keyboard',
        parameter_overrides=[Parameter('use_sim_time', value=True)],
    )
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    marker_publisher = WaypointMarkerPublisher(node)
    gazebo_marker_publisher = GazeboWaypointPublisher(node)
    saved_waypoints = load_waypoints(points_file)
    marker_publisher.publish(saved_waypoints)
    gazebo_marker_publisher.publish(saved_waypoints, tf_buffer)
    command_topic = TELEOP_CMD_TOPIC if collision_monitor else 'cmd_vel'
    if ROS_DISTRO == 'humble':
        pub = node.create_publisher(Twist, command_topic, qos)
    else:
        pub = node.create_publisher(TwistStamped, command_topic, qos)
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while (1):
            rclpy.spin_once(node, timeout_sec=0.1)  # Receives TF and /clock updates.
            key = get_key(settings)
            key = key.translate(RUSSIAN_TELEOP_KEYS)
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'g':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)

                pose = robot_pose(tf_buffer, node, 'map')
                if pose is not None:
                    gazebo_pose = gazebo_marker_publisher.robot_position()
                    point_name = input("Input the goal name end pres [Enter]: ").strip()
                    if point_name:
                        # Nav2 receives goals in `map`, so record the same frame.
                        with open(os.path.expanduser(points_file), 'a') as f:
                            line = (
                                f"{point_name}, {pose.translation.x}, {pose.translation.y}, "
                                f"{pose.rotation.z}, {pose.rotation.w}")
                            if gazebo_pose is not None:
                                line += f", {gazebo_pose.x}, {gazebo_pose.y}"
                            f.write(line + '\n')
                        from geometry_msgs.msg import PoseStamped
                        waypoint_pose = PoseStamped()
                        waypoint_pose.header.frame_id = 'map'
                        waypoint_pose.pose.position.x = pose.translation.x
                        waypoint_pose.pose.position.y = pose.translation.y
                        waypoint_pose.pose.orientation.z = pose.rotation.z
                        waypoint_pose.pose.orientation.w = pose.rotation.w
                        waypoint = {'name': point_name, 'pose': waypoint_pose}
                        if gazebo_pose is not None:
                            waypoint['gazebo_position'] = gazebo_pose
                            print(
                                f"Gazebo position for '{point_name}': "
                                f"{gazebo_pose.x}, {gazebo_pose.y}")
                        else:
                            print(
                                'Gazebo model pose is unavailable; no Gazebo marker will be '
                                'created. Restart S2_startSim.sh and teleop.')
                        saved_waypoints.append(waypoint)
                        marker_publisher.publish(saved_waypoints)
                        gazebo_marker_publisher.publish(saved_waypoints, tf_buffer)
                        print(f"Saved map waypoint '{point_name}':", pose.translation.x, pose.translation.y)
                    else:
                        print("Point name don`t enter.")
                else:
                    print("Map pose is unavailable. Start Cartographer or Nav2 and set 2D Pose Estimate first.")
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            if ROS_DISTRO == 'humble':
                twist = Twist()
                twist.linear.x = control_linear_velocity
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_velocity

                pub.publish(twist)
            else:
                twist_stamped = TwistStamped()
                twist_stamped.header.stamp = node.get_clock().now().to_msg()
                twist_stamped.header.frame_id = ''
                twist_stamped.twist.linear.x = control_linear_velocity
                twist_stamped.twist.linear.y = 0.0
                twist_stamped.twist.linear.z = 0.0

                twist_stamped.twist.angular.x = 0.0
                twist_stamped.twist.angular.y = 0.0
                twist_stamped.twist.angular.z = control_angular_velocity

                pub.publish(twist_stamped)

    except Exception as e:
        print(e)

    finally:
        if ROS_DISTRO == 'humble':
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
        else:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = node.get_clock().now().to_msg()
            twist_stamped.header.frame_id = ''
            twist_stamped.twist.linear.x = control_linear_velocity
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = 0.0
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = control_angular_velocity
            pub.publish(twist_stamped)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        stop_process(safety_manager)
        stop_process(safety_monitor)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Keyboard teleoperation and map waypoint recording for TurtleBot3.')
    parser.add_argument('points_file', nargs='?', default='~/points.txt',
                        help='Waypoint file (default: ~/points.txt)')
    parser.add_argument('--collision-monitor', action='store_true',
                        help='Filter teleop velocities through lidar-based slowdown and stop zones.')
    arguments = parser.parse_args()
    main(os.path.expanduser(arguments.points_file), arguments.collision_monitor)
