from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'SCRIPTS'))
try:
    import clear_navigation_trails as clear
except ImportError:
    clear = None


@unittest.skipIf(clear is None, 'Требуется окружение ROS 2 и gazebo_msgs')
class ClearTrailTests(unittest.TestCase):
    def test_only_trail_dots_are_deleted_and_future_runs_work(self):
        models=['burger','ground_plane','navigation_trail_12_1_1',
                'navigation_trail_12_1_2','navigation_trail_robot','cargo_load_1',
                'waypoint_p1','navigation_trail_12_1_3::link']
        deleted=[]
        def service(node, kind, message, timeout):
            if kind is clear.GetModelList:
                return SimpleNamespace(model_names=list(models),success=True)
            deleted.append(message.name);models.remove(message.name)
            return SimpleNamespace(success=True)
        with patch.object(clear,'request',side_effect=service):
            self.assertEqual(clear.clear_trails(None),2)
            self.assertEqual(clear.clear_trails(None),0)
            models.append('navigation_trail_12_2_1')
            self.assertEqual(clear.clear_trails(None),1)
        self.assertEqual(deleted,['navigation_trail_12_1_1','navigation_trail_12_1_2',
                                  'navigation_trail_12_2_1'])
        self.assertIn('burger',models)
        self.assertIn('cargo_load_1',models)

    def test_service_failure_is_reported(self):
        with patch.object(clear,'request',side_effect=TimeoutError('no Gazebo')):
            with self.assertRaises(TimeoutError):clear.clear_trails(None)
