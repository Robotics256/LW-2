"""Mixed CLI sequences: ordering, validation and aborts, with no broker."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'SCRIPTS'))
import sent_many_task_cli as cli
import sent_many_task


class SequenceCliTests(unittest.TestCase):
    def test_up_and_down_without_point_file(self):
        self.assertEqual(cli.parse_steps(['up','down'], 'missing.txt'),
                         [('cargo','load'),('cargo','unload')])

    def test_point_names_are_checked_before_connecting(self):
        with patch.object(cli, 'waypoint_names', return_value={'S2'}), \
             patch.object(cli, 'TaskClient') as factory:
            with self.assertRaises(SystemExit) as error:
                cli.main(['S2','missing','up'])
            self.assertEqual(error.exception.code, 2)
            factory.assert_not_called()

    def test_mixed_sequence_is_read_from_real_file(self):
        with tempfile.TemporaryDirectory() as directory:
            points=Path(directory)/'points.txt'
            points.write_text('S2,0,0,0,1\nT1,1,0,0,1\nT2,2,0,0,1\n')
            steps=cli.parse_steps(['S2','T1','up','T2','down','S2'],points)
        client=Mock()
        client.send_move_task.side_effect=[1,2,4,6]
        client.send_cargo_task.side_effect=[3,5]
        cli.run_sequence(client,steps,'orientation')
        from unittest.mock import call
        self.assertEqual(client.mock_calls,[
            call.send_move_task('S2','orientation'),call.wait_task_success(1,timeout=300),
            call.send_move_task('T1','orientation'),call.wait_task_success(2,timeout=300),
            call.send_cargo_task('load'),call.wait_task_success(3,timeout=30),
            call.send_move_task('T2','orientation'),call.wait_task_success(4,timeout=300),
            call.send_cargo_task('unload'),call.wait_task_success(5,timeout=30),
            call.send_move_task('S2','orientation'),call.wait_task_success(6,timeout=300)])

    def test_cargo_error_or_timeout_prevents_next_move_and_closes_client(self):
        for failure in (RuntimeError('status=2'),TimeoutError('no response')):
            with self.subTest(failure=failure), \
                 patch.object(cli,'waypoint_names',return_value={'T1'}), \
                 patch.object(cli,'TaskClient') as factory:
                factory.return_value.wait_task_success.side_effect=failure
                self.assertEqual(cli.main(['up','T1']),1)
                factory.return_value.send_move_task.assert_not_called()
                factory.return_value.close.assert_called_once()

    def test_missing_file_is_rejected_before_connecting(self):
        with patch.object(cli,'waypoint_names',side_effect=FileNotFoundError('missing')), \
             patch.object(cli,'TaskClient') as factory:
            with self.assertRaises(SystemExit):cli.main(['T1'])
            factory.assert_not_called()

    def test_cli_passes_custom_mqtt_settings(self):
        with patch.object(cli, 'TaskClient') as factory:
            self.assertEqual(cli.main(['up', '--broker', 'lab-broker', '--port', '1884',
                                       '--workspace', 'group42', '--name', 'robot42']), 0)
            factory.assert_called_once_with(broker='lab-broker', port=1884,
                                            workspace='group42', module_name='robot42')

    def test_editable_main_passes_settings_from_student_code(self):
        with patch.object(sent_many_task, 'TaskClient') as factory, \
             patch.multiple(sent_many_task, MQTT_BROKER='student-broker', MQTT_PORT=1884,
                            WORKSPACE='student-group', MODULE_NAME='student-robot'):
            self.assertEqual(sent_many_task.main(), 0)
            factory.assert_called_once_with(broker='student-broker', port=1884,
                                            workspace='student-group', module_name='student-robot')

    def test_main_success_closes_client(self):
        with patch.object(cli,'TaskClient') as factory:
            self.assertEqual(cli.main(['up','down']),0)
            self.assertEqual(factory.return_value.wait_task_success.call_count,2)
            factory.return_value.close.assert_called_once()

    def test_editable_main_stops_before_unload_if_loading_fails(self):
        with patch.object(sent_many_task,'TaskClient') as factory:
            factory.return_value.wait_task_success.side_effect=[None,None,RuntimeError('load failed')]
            self.assertEqual(sent_many_task.main(),1)
            factory.return_value.send_cargo_task.assert_called_once_with('load')
            self.assertEqual(factory.return_value.send_move_task.call_count,2)
            factory.return_value.close.assert_called_once()
