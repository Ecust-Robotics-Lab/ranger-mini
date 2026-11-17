import json,sys
from pathlib import Path
from dataclasses import replace
from functools import partial
import numpy as np
sys.path.insert(0,'ws_nav/src/ranger_navigation')
from ranger_navigation.exact_controller import ExactController
from ranger_navigation.geometry import Body
from ranger_navigation.models import Limits,Mode
from exact_mppi.mppi_jax.logger import logger
logger.setLevel(40)
p=np.load('docs/exact_mppi_evidence/live_changed_scene_20261110/speed_frozen_request.npz');meta=json.loads(str(p['meta']))
for change in ['temperature_005','temperature_002']:
 c=ExactController(Body(clearance=.03),Limits(speed=.3,yaw_rate=.2),visibility_reserve=.04);o=c.optimizers[Mode(meta['mode'])];crit=o.critic_manager_.upstream.critics_
 if change.startswith('temperature_'):o.settings_=replace(o.settings_,temperature=.05 if change.endswith('005') else .02)
 if change=='no_path_align':crit.pop('PathAlignCritic',None)
 if change=='no_path_follow':crit.pop('PathFollowCritic',None)
 if change=='no_visibility_preference':o.critic_manager_.reserve_half=o.critic_manager_.half
 if change=='goal_x5':
  score=crit['GoalCritic']['score'];crit['GoalCritic']['score']=partial(score.func,params=replace(score.keywords['params'],weight=50.))
 u=[]
 for i in range(5):
  r=c.compute(p['pose'],p['speed'],p['path'],meta['mode'],p['points'],(p['visibility_cells'],p['observer_pose']));u.append(r['command'].tolist())
 print(json.dumps({'change':change,'commands':u}),flush=True)
