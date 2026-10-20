"""ROS-independent state machine for the existing LW-2 JSON protocol.

All methods run on the ROS thread. The MQTT thread only enqueues messages.
Task.Id/Type/Data/Status retain their original meanings, including Busy=3.
"""

from collections import OrderedDict
from copy import deepcopy
import time

from navigation_modes import MODES


TASK_TYPES = {"point": "Point", "upload": "Upload", "load": "Upload",
              "download": "Download", "unload": "Download", "stop": "Stop"}


def task_id(value):
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ValueError("Task.Id must be an Int64")
    try:
        result = int(value)
    except ValueError as error:
        raise ValueError("Task.Id must be an Int64") from error
    if not -(2 ** 63) <= result < 2 ** 63 or result == -1:
        raise ValueError("Task.Id must be an Int64 other than -1")
    return result


class JsonTasks:
    def __init__(self, publish, default_mode="position", history_size=100, max_tasks=100):
        if default_mode not in MODES or history_size < 1 or max_tasks < 1:
            raise ValueError("Invalid mode or task storage size")
        self.publish = publish
        self.default_mode = default_mode
        self.history_size = history_size
        self.max_tasks = max_tasks
        self.tasks = OrderedDict()
        self.history = OrderedDict()
        self.head = None
        self.chain = False
        self.stop_task = None
        self.blocked = False
        self.loaded = False
        self.current_point = None

    @property
    def busy(self):
        return bool(self.tasks or self.head is not None or self.stop_task)

    def _reply_task(self, task, status, report):
        result = {key: deepcopy(value) for key, value in task.items() if not key.startswith("_")}
        result.update(Status=status, Report=report)
        self.publish({"Task": result})

    def _remember(self, task, status, report):
        now = time.time()
        result = {key: deepcopy(value) for key, value in task.items() if not key.startswith("_")}
        result.update(Status=status, Report=report, Loaded=self.loaded,
                      Duration=int(max(0, now - task.get("_accepted", now))),
                      CompletedAt=int(now * 1000))
        self.history[task["Id"]] = result
        while len(self.history) > self.history_size:
            self.history.popitem(last=False)
        self.publish({"Task": deepcopy(result)})

    def _known(self, identifier):
        if identifier in self.history:
            self.publish({"Task": deepcopy(self.history[identifier])})
            return True
        task = self.tasks.get(identifier)
        if task is None and self.stop_task and self.stop_task["Id"] == identifier:
            task = self.stop_task
        if task is not None:
            self._reply_task(task, 1, "task in progress" if task.get("_started") else "task accepted")
            return True
        return False

    def handle(self, data, uid=None):
        """Accept Module.Data and emit JSON response data, without executing ROS."""
        try:
            if not isinstance(data, dict):
                raise ValueError("Module.Data must be an object")
            request = data.get("Request")
            if request:
                self._request(request, data, uid)
                return
            raw = data.get("Tasks", [data.get("Task")])
            if "Tasks" in data and "Task" in data:
                raise ValueError("Use Task or Tasks, not both")
            if not isinstance(raw, list) or not raw or not all(isinstance(task, dict) for task in raw):
                raise ValueError("Task must be an object; Tasks must be a nonempty list")
            # A task containing just Id queries its saved or active result.
            if len(raw) == 1 and not raw[0].get("Type"):
                identifier = task_id(raw[0].get("Id"))
                if not self._known(identifier):
                    self._reply_task({"Id": identifier}, 3, "task not found")
                return
            self._submit(raw)
        except (ValueError, TypeError) as error:
            # Invalid packets never overwrite an existing task or result.
            raw = data.get("Tasks", [data.get("Task")]) if isinstance(data, dict) else []
            replied = False
            if isinstance(raw, list):
                for item in raw:
                    try:
                        identifier = task_id(item.get("Id"))
                    except (ValueError, TypeError, AttributeError):
                        continue
                    if identifier not in self.tasks and identifier not in self.history and (
                            not self.stop_task or self.stop_task["Id"] != identifier):
                        self._remember({"Id": identifier}, 2, str(error))
                        replied = True
            if not replied:
                self.publish({"RequestError": str(error), "RequestUID": uid})

    def _request(self, request, data, uid):
        if not isinstance(request, str):
            raise ValueError("Request must be status or history")
        request = request.strip().lower()
        if request == "status":
            info = {
                "DeviceStatus": 2 if self.blocked else int(self.busy),
                "Loaded": self.loaded, "CurrentPoint": self.current_point,
                "ActiveTaskIds": [key for key, task in self.tasks.items() if task.get("_started")],
                "PendingTaskIds": [key for key, task in self.tasks.items() if not task.get("_started")],
                "WaitingForTaskId": self.head if self.head is not None and self.head not in self.tasks else None,
                "Stopping": self.stop_task is not None,
            }
            self.publish({"StatusInfo": info, "RequestUID": uid})
        elif request == "history":
            limit = data.get("Limit", self.history_size)
            if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
                raise ValueError("Limit must be a positive integer")
            results = list(reversed(self.history.values()))[:min(limit, self.history_size)]
            self.publish({"History": {"capacity": self.history_size, "count": len(results),
                                      "Tasks": deepcopy(results)}, "RequestUID": uid})
        else:
            raise ValueError("Request must be status or history")

    def _normalize(self, raw):
        kind = raw.get("Type")
        kind = TASK_TYPES.get(kind.strip().lower()) if isinstance(kind, str) else None
        if kind is None:
            raise ValueError("Supported task types: Point, Upload, Download, Stop")
        task = {"Id": task_id(raw.get("Id")), "Type": kind, "Data": raw.get("Data", "")}
        if not isinstance(task["Data"], str):
            raise ValueError("Task.Data must be a string")
        task["Data"] = task["Data"].strip()
        if kind == "Point":
            if not task["Data"]:
                raise ValueError("Point requires a waypoint name in Data")
            mode = raw.get("Mode", self.default_mode)
            if mode not in MODES:
                raise ValueError("Mode must be position, orientation or proximity")
            task["Mode"] = mode
        elif "Mode" in raw:
            raise ValueError("Mode applies only to Point")
        if raw.get("NextTaskId") not in (None, -1, "-1"):
            task["NextTaskId"] = task_id(raw["NextTaskId"])
        return task

    def _submit(self, raw):
        tasks = [self._normalize(task) for task in raw]
        identifiers = [task["Id"] for task in tasks]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("Repeated task IDs in Tasks")
        known = [identifier in self.tasks or identifier in self.history or
                 (self.stop_task is not None and self.stop_task["Id"] == identifier)
                 for identifier in identifiers]
        if all(known):
            for identifier in identifiers:
                self._known(identifier)
            return
        if any(known):
            raise ValueError("Cannot mix known and new task IDs")
        if any(task["Type"] == "Stop" for task in tasks):
            if len(tasks) != 1 or "NextTaskId" in tasks[0]:
                raise ValueError("Stop must be a separate task without NextTaskId")
            reserved = {task.get("NextTaskId") for task in self.tasks.values()} | {self.head}
            if tasks[0]["Id"] in reserved:
                self._reply_task(tasks[0], 3, "task ID reserved by a chain; use a new Stop Id")
                return
            if self.stop_task:
                self._reply_task(tasks[0], 3, "Stop is already in progress")
                return
            self.stop_task = dict(tasks[0], _accepted=time.time(), _started=True)
            self._reply_task(tasks[0], 1, "Stop accepted")
            return
        if self.blocked or self.stop_task:
            self._reply_task(tasks[0], 3, "motion blocked; send a new Stop")
            return
        if len(tasks) + len(self.tasks) > self.max_tasks:
            raise ValueError("Task queue capacity exceeded")

        # An unlinked list is a route in array order. Explicit links define the
        # route independently of XML/JSON element order, as in AMR_sim.
        if len(tasks) > 1 and not any("NextTaskId" in task for task in tasks):
            for previous, following in zip(tasks, tasks[1:]):
                previous["NextTaskId"] = following["Id"]
        by_id = {task["Id"]: task for task in tasks}
        successors = [task["NextTaskId"] for task in tasks if "NextTaskId" in task]
        if len(successors) != len(set(successors)):
            raise ValueError("A task cannot have multiple predecessors")
        roots = set(identifiers) - set(successors)
        if len(roots) != 1:
            raise ValueError("Tasks must form one chain without cycles")
        root = roots.pop()
        seen, cursor = set(), root
        while cursor in by_id and cursor not in seen:
            seen.add(cursor)
            cursor = by_id[cursor].get("NextTaskId")
        if cursor in seen or len(seen) != len(tasks):
            raise ValueError("Tasks must form one connected chain without cycles")
        if any(identifier in self.tasks or identifier in self.history for identifier in successors):
            raise ValueError("NextTaskId refers to an existing task")

        parallel = (len(tasks) == 1 and not successors and not self.chain and
                    tasks[0]["Type"] in ("Upload", "Download"))
        successor = self.chain and root == self.head and self.head not in self.tasks
        # A successor can arrive before its predecessor finishes.
        if self.chain and not successor:
            reserved = [task.get("NextTaskId") for task in self.tasks.values()]
            successor = root in reserved
        if self.head is not None and not parallel and not successor:
            for task in tasks:
                self._reply_task(task, 3, "robot is busy")
            return
        for task in tasks:
            self.tasks[task["Id"]] = dict(task, _accepted=time.time(), _parallel=parallel)
            self._reply_task(task, 1, "task accepted")
        if not parallel:
            if self.head is None:
                self.head = root
            self.chain = self.chain or len(tasks) > 1 or bool(successors)

    def take_main(self):
        if self.blocked or self.stop_task:
            return None
        task = self.tasks.get(self.head)
        if task is not None and not task.get("_started"):
            task["_started"] = True
            return task
        return None

    def take_cargo(self):
        if self.blocked or self.stop_task:
            return None
        for task in self.tasks.values():
            if task.get("_parallel") and not task.get("_started"):
                task["_started"] = True
                return task
        return None

    def finish(self, identifier, success, report):
        task = self.tasks.pop(identifier, None)
        if task is None:
            return  # A late Gazebo callback must not resurrect a canceled task.
        if success and task["Type"] == "Point":
            self.current_point = task["Data"]
        if task.get("_parallel"):
            self._remember(task, 0 if success else 2, report)
            return
        if success:
            self.head = task.get("NextTaskId")
            if self.head is None:
                self.chain = False
            self._remember(task, 0, report)
        else:
            self.head = task.get("NextTaskId")
            self._remember(task, 2, report)
            self._cancel("chain canceled", include_parallel=False)

    def _cancel(self, report, include_parallel=True):
        reserved = {task["NextTaskId"] for task in self.tasks.values() if "NextTaskId" in task}
        if self.head is not None:
            reserved.add(self.head)
        reserved.difference_update(self.tasks)
        reserved.difference_update(self.history)
        self.head = None
        self.chain = False
        for identifier, task in list(self.tasks.items()):
            if include_parallel or not task.get("_parallel"):
                self.tasks.pop(identifier)
                self._remember(task, 2, report)
        for identifier in reserved:
            if identifier not in self.history and identifier not in self.tasks:
                self._remember({"Id": identifier}, 2, report)

    def finish_stop(self, success, report):
        task = self.stop_task
        if task is None:
            return
        self._cancel("canceled by Stop")
        self.stop_task = None
        self.blocked = not success
        self._remember(task, 0 if success else 2, report)
