"""Exercise S3 shutdown with a launcher that leaves a stubborn child behind."""

import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(Path('/opt/ros/humble/setup.bash').is_file(), 'requires ROS setup')
class NavigationShutdownTests(unittest.TestCase):
    def test_ctrl_c_stops_orphaned_container_and_removes_runtime_params(self):
        with tempfile.TemporaryDirectory(prefix='lw2_shutdown_') as directory:
            root = Path(directory)
            scripts = root / 'SCRIPTS'
            scripts.mkdir()
            for name in ('S3_startNav.sh', 'navigation_modes.py', 'navigation_controller.py', 'lw2_config.py', 'lw2_config.json', 'stop_previous_navigation.py'):
                shutil.copyfile(ROOT / 'SCRIPTS' / name, scripts / name)
            setup = root / 'ros2_ws/install/setup.bash'
            setup.parent.mkdir(parents=True)
            setup.write_text(f'export PATH={shlex.quote(str(root))}:"$PATH"\n')
            params = root / 'params'
            params.mkdir()
            (params / 'warehouse_burger.yaml').write_text(
                'bt_navigator:\n  ros__parameters: {}\namcl:\n  ros__parameters: {}\ncontroller_server:\n  ros__parameters:\n    proximity_goal_checker: {}\n')
            map_file = root / 'map.yaml'
            map_file.touch()
            marker_file = root / 'missing_points.txt'
            state_file = root / 'state.json'
            fake_ros = root / 'ros2'
            fake_ros.write_text('''#!/usr/bin/python3
import json, os, signal, subprocess, sys, time
from pathlib import Path
if sys.argv[1:3] == ['pkg', 'prefix']:
    print(os.environ['SHUTDOWN_FIXTURE_ROOT'])
    sys.exit(0)
child = subprocess.Popen([sys.executable, '-c', """
import os, signal, time
from pathlib import Path
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
Path(os.environ['SHUTDOWN_FIXTURE_ROOT'], 'child_ready').touch()
while True: time.sleep(.1)
"""])
# Simulate ros2 launch exiting on Ctrl+C without reaping its container.
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
runtime = next(arg.split(':=', 1)[1] for arg in sys.argv if arg.startswith('params_file:='))
Path(os.environ['SHUTDOWN_FIXTURE_ROOT'], 'state.json').write_text(
    json.dumps({'child': child.pid, 'group': os.getpgrp(), 'runtime': runtime}))
while True: time.sleep(.1)
''')
            fake_ros.chmod(0o755)
            env = dict(os.environ, SHUTDOWN_FIXTURE_ROOT=str(root), TURTLEBOT3_MODEL='burger')
            process = subprocess.Popen(
                ['bash', str(scripts / 'S3_startNav.sh'), str(map_file),
                 f'points_file:={marker_file}'], env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            state = None
            try:
                deadline = time.monotonic() + 10
                while not (root / 'child_ready').exists() or not state_file.exists():
                    if process.poll() is not None or time.monotonic() > deadline:
                        self.fail('Fixture did not start: ' + process.communicate(timeout=1)[0])
                    time.sleep(.05)
                state = json.loads(state_file.read_text())
                os.kill(process.pid, signal.SIGINT)
                output, _ = process.communicate(timeout=15)
                self.assertEqual(process.returncode, 130, output)
                self.assertFalse(Path(state['runtime']).exists(), output)
                # An orphan may briefly remain a zombie until its new parent reaps it.
                stat = Path(f"/proc/{state['child']}/stat")
                self.assertTrue(not stat.exists() or stat.read_text().split(') ')[1][0] == 'Z', output)
            finally:
                if state is not None:
                    try:
                        os.killpg(state['group'], signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                if process.poll() is None:
                    process.kill()
                    process.communicate(timeout=2)


if __name__ == '__main__':
    unittest.main()
