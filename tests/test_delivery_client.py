"""Delivery client checks without connecting to a real MQTT broker or ROS."""

import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'SCRIPTS'))
from task_client import TaskClient, MODULE_NAME, MODULE_TYPE, WORKSPACE
from mqtt_tasks import JsonTasks


class DeliveryClientTests(unittest.TestCase):
    def setUp(self):
        self.client = TaskClient.__new__(TaskClient)
        self.client.module_name = MODULE_NAME
        self.client.module_type = MODULE_TYPE
        self.client.command_topic = f"{WORKSPACE}/devices/in/{MODULE_TYPE}/{MODULE_NAME}"
        self.client.status_topic = f"{WORKSPACE}/devices/out/{MODULE_TYPE}/{MODULE_NAME}"
        self.client._lock = threading.Lock()
        self.client._status_event = threading.Event()
        self.client._task_status = {}
        self.client.client = Mock()

    def reply(self, identifier, status, loaded=None):
        task = {'Id': identifier, 'Status': status, 'Loaded': loaded}
        self.client.on_message(None, None, SimpleNamespace(payload=json.dumps({
            'Module': {'Name': MODULE_NAME, 'Data': {'Task': task}}
        }).encode()))

    def test_cargo_payload_can_be_executed_by_the_driver_task_queue(self):
        for action, expected in [('load', 'Upload'), ('unload', 'Download')]:
            with self.subTest(action=action):
                identifier = self.client.send_cargo_task(action)
                call = self.client.client.publish.call_args
                payload = json.loads(call.args[1])
                self.assertEqual(payload['Module']['Data']['Task']['Type'], expected)
                self.assertEqual(call.kwargs, {'qos': 2, 'retain': False})
                queue = JsonTasks(lambda data: None)
                queue.handle(payload['Module']['Data'])
                task = queue.take_cargo()
                self.assertEqual(task['Id'], identifier)
                self.assertEqual(task['Type'], expected)
                self.client.client.publish.return_value.wait_for_publish.assert_called()

    def test_load_then_unload_returns_confirmed_loaded_state(self):
        for action, loaded in [('load', True), ('unload', False)]:
            identifier = self.client.send_cargo_task(action)
            self.reply(identifier, 0, loaded)
            result = self.client.wait_task_success(identifier, timeout=.1)
            self.assertIs(result['loaded'], loaded)

    def test_arrival_does_not_complete_a_different_cargo_task(self):
        self.reply(51, 0)
        with self.assertRaises(TimeoutError):
            self.client.wait_task_success(52, timeout=.001)

    def test_accepted_cargo_is_not_reported_as_completed(self):
        self.reply(52, 1)
        with self.assertRaises(TimeoutError):
            self.client.wait_task_success(52, timeout=.001)

    def test_failed_or_rejected_cargo_interrupts_delivery(self):
        for status in (2, 3):
            self.reply(52, status)
            with self.assertRaisesRegex(Exception, f'status={status}'):
                self.client.wait_task_success(52, timeout=.1)

    def test_custom_settings_apply_to_connection_topics_payloads_and_replies(self):
        transport = Mock()
        transport.loop_start.side_effect = lambda: transport.on_subscribe(None, None, 1, [1])
        with patch('task_client.mqtt.Client', return_value=transport):
            client = TaskClient(broker='lab-broker', port=1884, workspace='group42',
                                module_name='robot42', module_type='delivery')
        transport.connect.assert_called_once_with('lab-broker', 1884, 60)
        client.on_connect(transport, None, {}, 0)
        transport.subscribe.assert_called_once_with('group42/devices/out/delivery/robot42', qos=1)
        for send, argument, task_type in [(client.send_move_task, 'A2', 'Point'),
                                          (client.send_cargo_task, 'load', 'Upload'),
                                          (client.send_cargo_task, 'unload', 'Download')]:
            identifier = send(argument)
            published = transport.publish.call_args
            self.assertEqual(published.args[0], 'group42/devices/in/delivery/robot42')
            payload = json.loads(published.args[1])
            self.assertEqual(payload['Module']['Name'], 'robot42')
            self.assertEqual(payload['Module']['Data']['Task']['Type'], task_type)
            for name in (MODULE_NAME, 'robot42'):
                reply = {'Module': {'Name': name, 'Data': {'Task': {'Id': identifier, 'Status': 0}}}}
                client.on_message(None, None, SimpleNamespace(payload=json.dumps(reply).encode()))
                self.assertEqual(identifier in client._task_status, name == 'robot42')
            self.assertEqual(client.wait_task_success(identifier, timeout=.1)['status'], 0)
        client.close()
        transport.loop_stop.assert_called_once()
        transport.disconnect.assert_called_once()

    def test_invalid_cargo_does_not_publish(self):
        with self.assertRaises(ValueError):
            self.client.send_cargo_task('Point')
        self.client.client.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
