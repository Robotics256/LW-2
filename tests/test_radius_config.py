"""Validate JSON radius and its application to the actual Nav2 parameters."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

repo=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(repo/'SCRIPTS'))
from lw2_config import apply_navigation_config, load_navigation_config

class RadiusConfigTests(unittest.TestCase):
    def test_default_radius(self):
        config=load_navigation_config()
        expected=json.loads((repo/'SCRIPTS/lw2_config.json').read_text())['navigation']['proximity_radius']
        self.assertEqual(config['proximity_radius'],expected)
        self.assertGreater(config['proximity_radius'],0)

    def test_custom_radius_changes_only_proximity_for_all_models(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Path(directory)/'config.json'
            config.write_text(json.dumps({'navigation':{'proximity_radius':0.17}}))
            for model in ('burger','waffle','waffle_pi'):
                source=repo/f'ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/params/warehouse_{model}.yaml'
                params=yaml.safe_load(source.read_text())
                before=json.loads(json.dumps(params))
                apply_navigation_config(params,config)
                expected=before['controller_server']['ros__parameters']['proximity_goal_checker']
                expected['xy_goal_tolerance']=0.17
                self.assertEqual(params,before)

    def test_invalid_radius_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Path(directory)/'config.json'
            for value in (0,-1,True,'0.3',None,float('nan'),float('inf')):
                config.write_text(json.dumps({'navigation':{'proximity_radius':value}}))
                with self.subTest(value=value),self.assertRaises(ValueError):load_navigation_config(config)
            config.write_text('{}')
            with self.assertRaises(ValueError):load_navigation_config(config)

if __name__=='__main__':unittest.main()
