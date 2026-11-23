from pathlib import Path
import sys,json,time,numpy as np
root=Path('/home/lab307/ranger_nav');sys.path.insert(0,str(root/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import load_geometry,densify,validate_sweep
from ranger_navigation.models import Mode,rollout,braking_commands
from ranger_navigation.planner import Grid
from ranger_navigation.coverage_policy import CoveragePolicy
from ranger_navigation.guidance import observed_prefix
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.exact_controller import ExactController
out=root/'docs/workbench_evidence/20261117/coverage_margin'
with np.load(out/'original_request.npz',allow_pickle=False) as a:pose=a['pose'];observer=a['observer_pose'];grid=Grid(a['visibility_cells'],.05,(-2,-2,0));points=a['points']
with np.load(out/'original_result.npz',allow_pickle=False) as a:path=a['segment_0']
body,_,_=load_geometry(root/'config/fast_livo2/navigation_prep.yaml',.04)
old,old_diagnostic=observed_prefix(densify(path,body,.025),pose,np.zeros(3),observer,grid,body,CoveragePolicy(.055,.055,0))
prefix,new_diagnostic=observed_prefix(densify(path,body,.025),pose,np.zeros(3),observer,grid,body,CoveragePolicy())
assert old is None and prefix is not None
sampler=ObstacleMemory(body);sampler.pose=pose
sampled=sampler._sample_points(points,512)
controller=ExactController(body,samples=256,coverage_policy=CoveragePolicy());warmup=controller.prewarm()
began=time.monotonic();output=controller.compute(pose,np.zeros(3),prefix,Mode.PARALLEL,sampled,(grid.cells,observer));elapsed=time.monotonic()-began
memory=ObstacleMemory(body);memory.set_visibility(observer,np.full(180,8.),grid.cells)
tail=rollout(output['trajectory'][-1],braking_commands(output['controls'][-1],controller.limits),.05)
okay,minimum,reason=validate_sweep(np.vstack((output['trajectory'],tail[1:])),points,body,coverage=memory.covers,margin=body.clearance+.04,coverage_margin=.025)
assert okay,reason
assert np.linalg.norm(output['command'][:2])>1e-3,output['command']
np.savez(out/'counterfactual_trajectory.npz',trajectory=output['trajectory'],controls=output['controls'],points=points,coverage=grid.cells,observer=observer)
result={'passed':True,'kind':'recorded_input_counterfactual_not_live','initial_velocity_assumption':[0,0,0],
 'basis':'stationary wheel/zero commands in this recorded task; no robot command sent',
 'old':old_diagnostic,'new':new_diagnostic,'command':output['command'].tolist(),
 'prefix_length_m':float(np.linalg.norm(np.diff(prefix[:,:2],axis=0),axis=1).sum()),
 'independent_full_points_and_braking_validation':{'passed':okay,'reason':reason,'minimum_distance_m':minimum},
 'compute_seconds':elapsed,'warmup_seconds':warmup}
(out/'recorded_control.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
