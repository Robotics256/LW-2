"""Stop previous Nav2 runs for this workspace, user and ROS domain."""

import os
from pathlib import Path
import signal
import sys
import time
from dataclasses import dataclass


@dataclass
class Process:
    pid: int
    parent: int
    started: str
    args: list
    environment: dict


def read_process(pid):
    directory = Path('/proc') / str(pid)
    try:
        if directory.stat().st_uid != os.getuid():
            return None
        fields = (directory / 'stat').read_text().rsplit(') ', 1)[1].split()
        if fields[0] == 'Z':
            return None
        args = (directory / 'cmdline').read_bytes().decode(errors='replace').split('\0')
        environment = dict(
            item.split('=', 1) for item in
            (directory / 'environ').read_bytes().decode(errors='replace').split('\0')
            if '=' in item)
        return Process(pid, int(fields[1]), fields[19], [arg for arg in args if arg], environment)
    except (OSError, ValueError, IndexError):
        return None


def snapshot():
    processes = {}
    for directory in Path('/proc').iterdir():
        if directory.name.isdigit():
            process = read_process(int(directory.name))
            if process:
                processes[process.pid] = process
    return processes


def find_navigation(processes, project, domain, current_pid):
    """Match navigation roots, including containers whose launcher has exited."""
    excluded = set()
    pid = current_pid
    while pid in processes and pid not in excluded:
        excluded.add(pid)
        pid = processes[pid].parent
    project = Path(project).resolve()
    install = project / 'ros2_ws/install'
    roots = set()
    for pid, process in processes.items():
        if pid in excluded or process.environment.get('ROS_DOMAIN_ID', '0') != domain:
            continue
        args = process.args
        workspace = any(
            Path(prefix) == install or install in Path(prefix).parents
            for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH')
            for prefix in process.environment.get(key, '').split(':') if prefix)
        wrapper = any(arg == str(project / 'SCRIPTS/S3_startNav.sh') or (
            arg.endswith('/SCRIPTS/S3_startNav.sh') and
            Path(f'/proc/{pid}/cwd', arg).resolve() == project / 'SCRIPTS/S3_startNav.sh')
            for arg in args)
        launcher = any(
            Path(arg).name == 'ros2' and args[index + 1:index + 4] ==
            ['launch', 'turtlebot3_navigation2', 'navigation2.launch.py']
            for index, arg in enumerate(args))
        workspace = workspace or process.environment.get('TB3_NAV_PROJECT_DIR') == str(project)
        rviz = any(arg == str(install / 'turtlebot3_navigation2/share/'
                              'turtlebot3_navigation2/rviz/tb3_navigation2.rviz') or
                   arg == str(project / 'ros2_ws/src/turtlebot3/'
                              'turtlebot3_navigation2/rviz/tb3_navigation2.rviz')
                   for arg in args)
        markers = str(project / 'SCRIPTS/show_waypoints.py') in args and '--rviz-only' in args
        container = workspace and any(Path(arg).name == 'component_container_isolated'
                                      for arg in args) and '__node:=nav2_container' in args
        node_packages = {'nav2_amcl', 'nav2_map_server', 'nav2_controller', 'nav2_planner',
                         'nav2_smoother', 'nav2_behaviors', 'nav2_bt_navigator',
                         'nav2_waypoint_follower', 'nav2_velocity_smoother'}
        standalone = workspace and any(Path(arg).parent.name in node_packages and
                                      '/lib/' in arg for arg in args[:1])
        if wrapper or (launcher and workspace) or rviz or markers or container or standalone:
            roots.add(pid)
    targets = set(roots)
    while True:
        descendants = {pid for pid, process in processes.items()
                       if process.parent in targets and pid not in excluded}
        expanded = targets | descendants
        if expanded == targets:
            return {pid: processes[pid] for pid in targets}
        targets = expanded


def remaining(processes):
    return {pid: old for pid, old in processes.items()
            if (current := read_process(pid)) and current.started == old.started}


def stop_processes(processes, grace=5.0, terminate_grace=2.0):
    """Bound shutdown time and check process identity before every signal."""
    pending = remaining(processes)
    for signum, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, terminate_grace),
                            (signal.SIGKILL, 1.0)):
        if not pending:
            return True
        if signum == signal.SIGKILL:
            print('Принудительная остановка зависших PID: ' +
                  ', '.join(map(str, sorted(pending))), flush=True)
        for pid in pending:
            if pid in remaining({pid: pending[pid]}):
                try:
                    os.kill(pid, signum)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        while pending and time.monotonic() < deadline:
            time.sleep(.1)
            pending = remaining(pending)
    return not pending


def main(project):
    processes = find_navigation(snapshot(), project, os.environ.get('ROS_DOMAIN_ID', '0'), os.getpid())
    if not processes:
        print('Предыдущих процессов навигации LW-2 не найдено.', flush=True)
        return 0
    print('Остановка предыдущей навигации LW-2, PID: ' +
          ', '.join(map(str, sorted(processes))), flush=True)
    if not stop_processes(processes):
        print('Не удалось остановить предыдущую навигацию; новый запуск отменён.', file=sys.stderr)
        return 1
    print('Предыдущая навигация остановлена.', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1]))
