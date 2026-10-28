"""Task scenarios without a broker, Gazebo or ROS."""

from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "SCRIPTS"))
from mqtt_tasks import JsonTasks
from send_command import build_command_payload
from sent_one_task import build_task_payload as single_payload
from task_client import build_task_payload as route_payload


def point(identifier, name="parking", **fields):
    return dict(Id=identifier, Type="Point", Data=name, **fields)


class JsonCommandTests(unittest.TestCase):
    def setUp(self):
        self.responses = []
        self.service = JsonTasks(lambda data: self.responses.append(deepcopy(data)), history_size=3)

    def submit(self, task):
        self.service.handle({"Task": task})

    def test_status_is_available_during_motion_without_changing_the_goal(self):
        self.submit(point(1))
        self.service.take_main()
        self.service.handle({"Request": "status"}, uid=42)
        response = self.responses[-1]
        self.assertEqual(response["RequestUID"], 42)
        self.assertEqual(response["StatusInfo"]["ActiveTaskIds"], [1])
        self.assertEqual(response["StatusInfo"]["DeviceStatus"], 1)
        self.assertEqual(self.service.head, 1)
        self.service.finish(1, True, "arrived")
        self.service.handle({"Request": "status"})
        self.assertEqual(self.responses[-1]["StatusInfo"]["CurrentPoint"], "parking")
        self.assertEqual(self.responses[-1]["StatusInfo"]["DeviceStatus"], 0)

    def test_default_proximity_and_explicit_modes(self):
        for index, mode in enumerate((None, "position", "orientation", "proximity"), 1):
            self.submit(point(index, **({"Mode": mode} if mode else {})))
            self.assertEqual(self.service.take_main()["Mode"], mode or "proximity")
            self.service.finish(index, True, "arrived")

    def test_history_is_bounded_newest_first_and_excludes_active_tasks(self):
        for identifier in range(1, 5):
            self.submit(point(identifier))
            self.service.take_main()
            self.service.finish(identifier, identifier != 3, "completed")
        self.submit(point(5))
        self.service.handle({"Request": "history", "Limit": 2})
        history = self.responses[-1]["History"]
        self.assertEqual(history["capacity"], 3)
        self.assertEqual([task["Id"] for task in history["Tasks"]], [4, 3])
        self.assertEqual(history["Tasks"][1]["Status"], 2)
        self.assertIn("CompletedAt", history["Tasks"][0])
        self.assertNotIn(1, self.service.history)

    def test_invalid_history_limits_do_not_change_state(self):
        for limit in (0, -1, True, 1.5, "5"):
            self.service.handle({"Request": "history", "Limit": limit})
            self.assertIn("RequestError", self.responses[-1])

    def test_chain_runs_on_success_and_cancels_remaining_steps_on_error(self):
        self.service.handle({"Tasks": [point(1), {"Id": 2, "Type": "Upload"}, point(3)]})
        self.assertEqual(self.service.take_main()["Id"], 1)
        self.assertIsNone(self.service.take_main())
        self.service.finish(1, True, "arrived")
        self.assertEqual(self.service.take_main()["Id"], 2)
        self.service.finish(2, False, "Gazebo unavailable")
        self.assertIsNone(self.service.take_main())
        self.assertEqual(self.service.history[3]["Status"], 2)
        self.assertFalse(self.service.busy)

    def test_explicit_chain_links_define_order(self):
        self.service.handle({"Tasks": [point(2), point(1, NextTaskId=2)]})
        self.assertEqual(self.service.take_main()["Id"], 1)
        self.service.finish(1, True, "arrived")
        self.assertEqual(self.service.take_main()["Id"], 2)

    def test_separately_delivered_successors_and_waiting_can_be_stopped(self):
        self.submit(point(1, NextTaskId=2))
        self.service.take_main()
        self.submit(point(2, NextTaskId=3))
        self.service.finish(1, True, "arrived")
        self.assertEqual(self.service.take_main()["Id"], 2)
        self.service.finish(2, True, "arrived")
        self.assertTrue(self.service.busy)
        self.service.handle({"Request": "status"})
        self.assertEqual(self.responses[-1]["StatusInfo"]["WaitingForTaskId"], 3)
        self.submit({"Id": 4, "Type": "Stop"})
        self.service.finish_stop(True, "robot stopped")
        self.assertEqual(self.service.history[3]["Status"], 2)
        self.assertFalse(self.service.busy)

    def test_stop_cancels_goal_queue_and_ignores_late_cargo_completion(self):
        self.submit(point(1))
        self.service.take_main()
        self.submit({"Id": 2, "Type": "Upload"})
        self.service.take_cargo()
        self.submit({"Id": 3, "Type": "Stop"})
        self.service.finish_stop(True, "robot stopped")
        self.service.finish(2, True, "late callback")
        self.assertEqual(self.service.history[1]["Status"], 2)
        self.assertEqual(self.service.history[2]["Status"], 2)
        self.assertEqual(self.service.history[3]["Status"], 0)
        self.submit(point(4))
        self.assertEqual(self.service.take_main()["Id"], 4)

    def test_failed_stop_blocks_motion_until_successful_retry(self):
        self.submit(point(1))
        self.submit({"Id": 2, "Type": "Stop"})
        self.service.finish_stop(False, "cancel timeout")
        self.submit(point(3))
        self.assertEqual(self.responses[-1]["Task"]["Status"], 3)
        self.assertIsNone(self.service.take_main())
        self.submit({"Id": 4, "Type": "Stop"})
        self.service.finish_stop(True, "robot stopped")
        self.submit(point(5))
        self.assertEqual(self.service.take_main()["Id"], 5)

    def test_repeated_task_id_returns_the_same_result_without_executing(self):
        self.submit(point(1))
        self.service.take_main()
        self.service.finish(1, True, "arrived")
        result = deepcopy(self.service.history[1])
        self.submit(point(1))
        self.assertEqual(self.responses[-1]["Task"], result)
        self.assertIsNone(self.service.take_main())

    def test_bad_chain_is_rejected_before_any_task_starts(self):
        self.service.handle({"Tasks": [point(1, NextTaskId=2), point(2, NextTaskId=1)]})
        self.assertIsNone(self.service.take_main())
        self.assertEqual(self.service.history[1]["Status"], 2)
        self.assertFalse(self.service.tasks)

    def test_mixed_known_ids_do_not_overwrite_current_task(self):
        self.submit(point(1))
        self.service.take_main()
        self.service.handle({"Tasks": [point(1), point(2)]})
        self.assertEqual(self.service.tasks[1]["Type"], "Point")
        self.assertNotIn(1, self.service.history)
        self.assertEqual(self.service.history[2]["Status"], 2)

    def test_manual_cargo_remains_available_while_a_point_is_running(self):
        self.submit(point(1))
        self.service.take_main()
        self.submit({"Id": 2, "Type": "Upload"})
        self.assertEqual(self.service.take_cargo()["Id"], 2)
        self.assertEqual(self.service.head, 1)

    def test_new_sender_preserves_json_contract(self):
        request = build_command_payload("history", limit=5)["Module"]
        self.assertEqual(request["Type"], "delivery")
        self.assertEqual(request["Data"], {"Request": "history", "Limit": 5})
        stop = build_command_payload("Stop", identifier=42)["Module"]["Data"]["Task"]
        self.assertEqual(stop, {"Id": 42, "Type": "Stop", "Data": ""})

    def test_senders_keep_driver_default_and_allow_per_task_mode(self):
        for build in (single_payload, route_payload):
            _, default = build("parking")
            self.assertNotIn("Mode", default["Module"]["Data"]["Task"])
            _, oriented = build("parking", "orientation")
            self.assertEqual(oriented["Module"]["Data"]["Task"]["Mode"], "orientation")


if __name__ == "__main__":
    unittest.main()
