"""One-shot static planner worker; never imports ROS/JAX or controls hardware."""
import json
import sys
from pathlib import Path
from workbench.core import ROOT, pose
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'ws_nav/src/ranger_navigation')]
from prepare_exact_field_test import check_leg
from ranger_navigation.geometry import load_geometry
from ranger_navigation.models import Limits, Mode
from ranger_navigation.planner import Grid
from fast_livo2.navigation.asset import load_map
import yaml

def preview(data):
    folder=Path(data['folder']); load_map(folder)
    cfg=yaml.safe_load((ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())
    profile=cfg['clearance_profiles'][data.get('profile','standard')]
    body,_,_=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml',profile['clearance'])
    result=check_leg(Grid.from_yaml(folder/'map.yaml'),body,Limits(speed=cfg['motion']['max_speed'],yaw_rate=cfg['motion']['max_yaw_rate'],acceleration=cfg['motion']['acceleration'],yaw_acceleration=cfg['motion']['yaw_acceleration'],min_radius=cfg['motion']['min_radius']),
        pose(data['start']),pose(data['goal']),Mode(int(data.get('mode',0))),budget=5.,
        static_reserve=profile['static_map_reserve'],static_clearance=profile['static_map_clearance'],prefer_parallel_in_narrow=profile['prefer_parallel_in_narrow'])
    return {**result,'static_only':True,'profile':data.get('profile','standard')}

if __name__=='__main__':
    try: print(json.dumps(preview(json.load(sys.stdin)),allow_nan=False))
    except Exception as exc: print(json.dumps({'accepted':False,'static_only':True,'reason':str(exc)})); sys.exit(2)
