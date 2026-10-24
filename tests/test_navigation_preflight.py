"""Check navigation process selection and shutdown before a replacement run."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'SCRIPTS'))
from stop_previous_navigation import Process, find_navigation, read_process, stop_processes


class PreflightSelectionTests(unittest.TestCase):
    def test_orphaned_navigation_is_selected_without_gazebo_or_other_domains(self):
        project = Path('/tmp/lw2_selection_fixture')
        env = {'AMENT_PREFIX_PATH': str(project / 'ros2_ws/install/turtlebot3_gazebo')}
        nav = ['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py']
        container = ['/opt/ros/humble/lib/rclcpp_components/component_container_isolated',
                     '-r', '__node:=nav2_container']
        processes = {
            1: Process(1, 0, '1', ['bash'], env),
            2: Process(2, 1, '2', ['python3', 'stop_previous_navigation.py'], env),
            3: Process(3, 1, '3', nav, env),
            4: Process(4, 3, '4', ['rviz2'], env),
            5: Process(5, 1, '5', container, env),
            6: Process(6, 1, '6', ['gzserver'], env),
            7: Process(7, 1, '7', container, dict(env, ROS_DOMAIN_ID='187')),
            8: Process(8, 1, '8', container, {'AMENT_PREFIX_PATH': '/other/install'}),
            9: Process(9, 1, '9', ['python3', str(project / 'SCRIPTS/show_waypoints.py'),
                                  'points.txt'], env),
            10: Process(10, 1, '10', ['python3', str(project / 'SCRIPTS/show_waypoints.py'),
                                     'points.txt', '--rviz-only'], env),
        }
        self.assertEqual(set(find_navigation(processes, project, '0', 2)), {3, 4, 5, 10})

    def test_current_invocation_and_its_ancestors_are_excluded(self):
        project = Path('/tmp/lw2_selection_fixture')
        env = {'TB3_NAV_PROJECT_DIR': str(project)}
        args = ['bash', str(project / 'SCRIPTS/S3_startNav.sh'), 'map.yaml']
        processes = {1: Process(1, 0, '1', args, env),
                     2: Process(2, 1, '2', ['python3', 'stop_previous_navigation.py'], env),
                     3: Process(3, 0, '3', args, env)}
        self.assertEqual(set(find_navigation(processes, project, '0', 2)), {3})

    def test_dead_launcher_does_not_hide_its_orphaned_container(self):
        project = Path('/tmp/lw2_selection_fixture')
        env = {'TB3_NAV_PROJECT_DIR': str(project)}
        process = Process(10, 1, '10', [
            '/opt/ros/humble/lib/rclcpp_components/component_container_isolated',
            '-r', '__node:=nav2_container'], env)
        self.assertEqual(set(find_navigation({10: process}, project, '0', 999)), {10})


class PreflightShutdownTests(unittest.TestCase):
    def test_previous_launcher_and_stubborn_child_stop_while_simulator_survives(self):
        with tempfile.TemporaryDirectory(prefix='lw2_preflight_') as directory:
            project = Path(directory)
            launcher = project / 'ros2'
            launcher.write_text('''import os, signal, subprocess, sys, time
from pathlib import Path
subprocess.Popen([sys.executable, '-c', """
import os, signal, time
from pathlib import Path
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
Path(os.environ['TB3_NAV_PROJECT_DIR'], 'child.pid').write_text(str(os.getpid()))
while True: time.sleep(.1)
"""])
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
while True: time.sleep(.1)
''')
            env = dict(os.environ, TB3_NAV_PROJECT_DIR=str(project), ROS_DOMAIN_ID='196')
            old = subprocess.Popen([sys.executable, str(launcher), 'launch',
                                    'turtlebot3_navigation2', 'navigation2.launch.py'],
                                   env=env, start_new_session=True,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            simulator = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                         env=env, start_new_session=True)
            child = None
            try:
                deadline = time.monotonic() + 5
                while not (project / 'child.pid').exists():
                    if time.monotonic() > deadline:
                        self.fail('Child did not start')
                    time.sleep(.05)
                child = int((project / 'child.pid').read_text())
                result = subprocess.run(
                    [sys.executable, str(ROOT / 'SCRIPTS/stop_previous_navigation.py'), str(project)],
                    env=env, capture_output=True, text=True, timeout=12)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('Принудительная остановка', result.stdout)
                old.wait(timeout=1)
                self.assertIsNone(read_process(child), result.stdout)
                self.assertIsNone(simulator.poll())
            finally:
                for pid in (old.pid, child, simulator.pid):
                    if pid:
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                old.wait(timeout=2)
                simulator.wait(timeout=2)


if __name__ == '__main__':
    unittest.main()
