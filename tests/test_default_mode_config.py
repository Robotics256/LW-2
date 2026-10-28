from pathlib import Path
import json,shutil,subprocess,sys,tempfile,unittest
repo=Path(__file__).resolve().parents[1]


class DefaultModeConfigTests(unittest.TestCase):
    def test_each_default_reaches_direct_cli_and_mqtt_queue_with_explicit_override(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            for name in ('navigation_modes.py','lw2_config.py'):
                shutil.copyfile(repo/'SCRIPTS'/name,folder/name)
            for mode in ('orientation','position','proximity'):
                (folder/'lw2_config.json').write_text(json.dumps({'navigation':{
                    'default_mode':mode,'proximity_radius':0.19}}))
                code='''import sys,argparse
sys.path[:0]=[sys.argv[1],sys.argv[2]]
from navigation_modes import DEFAULT_MODE,add_mode_arguments
from mqtt_tasks import JsonTasks
parser=argparse.ArgumentParser();add_mode_arguments(parser)
assert DEFAULT_MODE==sys.argv[3]
assert parser.parse_args([]).navigation_mode==sys.argv[3]
assert parser.parse_args(['--mode','position']).navigation_mode=='position'
queue=JsonTasks(lambda x:None)
queue.handle({'Task':{'Id':1,'Type':'Point','Data':'p1'}})
assert queue.take_main()['Mode']==sys.argv[3]
queue=JsonTasks(lambda x:None)
queue.handle({'Task':{'Id':1,'Type':'Point','Data':'p1','Mode':'orientation'}})
assert queue.take_main()['Mode']=='orientation'
'''
                subprocess.run([sys.executable,'-c',code,str(folder),str(repo/'SCRIPTS'),mode],check=True)

    def test_invalid_modes_rejected(self):
        sys.path.insert(0,str(repo/'SCRIPTS'))
        from lw2_config import load_navigation_config
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            for mode in ('wrong',None,True,1,[],{}):
                path.write_text(json.dumps({'navigation':{'default_mode':mode,'proximity_radius':0.3}}))
                with self.subTest(mode=mode),self.assertRaises(ValueError):load_navigation_config(path)
