"""Real failed request and layer-specific rejection; no hardware or ROS."""
from pathlib import Path
import sys,json
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.planner import Planner,Grid,NoPath
from ranger_navigation.geometry import Body,validate_sweep
from ranger_navigation.task import Task

def test_recorded_endpoint_failure_explains_both_ends_after_clearance_change():
    path=ROOT/'tests/fixtures/goal_freshness/original_request.npz'
    with np.load(path,allow_pickle=False) as data:
        meta=json.loads(str(data['meta']));grid=Grid(data['cells'],meta['resolution'],meta['origin']).with_obstacles(data['points'])
        start,goal=data['pose'],data['goal']
    with pytest.raises(NoPath) as old:
        Planner(Body(),static_reserve=.04).plan(grid,start,goal)
    assert old.value.details['start']['physical_static_free']
    assert str(old.value)=='start_static_clearance_insufficient'
    profile=yaml.safe_load((ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())['clearance_profiles']['standard']
    with pytest.raises(NoPath) as new:
        Planner(Body(clearance=profile['clearance']),static_reserve=profile['static_map_reserve'],static_clearance=profile['static_map_clearance']).plan(grid,start,goal)
    assert new.value.details['start']['free']
    assert str(new.value)=='goal_live_obstacle_collision'
    assert new.value.details['goal']['static_free'] and not new.value.details['goal']['dynamic_free']
    assert np.min(Body().distances(goal,grid.dynamic_points))<0

def test_25mm_static_total_is_used_by_planner_and_final_sweep_only():
    cells=np.full((140,500),100);cells[40:100,:]=0
    grid=Grid(cells,.01,(-2.5,-.7,0));body=Body(clearance=.04)
    profile=yaml.safe_load((ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())['clearance_profiles']['standard']
    planner=Planner(body,static_clearance=profile['static_map_clearance'])
    assert abs(planner.map_clearance+.015-.025)<1e-9
    segments=planner.plan(grid,[-.5,0,0],[.5,0,0])
    assert segments
    for segment in segments:
        assert validate_sweep(segment.poses,np.empty((0,2)),body,grid=grid,map_margin=planner.map_clearance)[0]
    with pytest.raises(NoPath):Planner(body,static_clearance=.04).plan(grid,[-.5,0,0],[.5,0,0])
    with pytest.raises(NoPath,match='start_live_obstacle_collision'):
        planner.plan(grid.with_obstacles([[-.5,.29]]),[-.5,0,0],[.5,0,0])
    assert body.length==.74 and body.width==.52 and body.clearance==.04

def test_static_physical_unknown_is_not_confused_with_extra_clearance():
    cells=np.zeros((80,80));cells[40,40]=-1;grid=Grid(cells,.05,(-2,-2,0))
    with pytest.raises(NoPath,match='start_static_collision_or_unknown'):
        Planner(static_reserve=0.).plan(grid,[0,0,0],[1,1,0])

def test_planning_cause_survives_input_pause_and_final_stall():
    task=Task();task.begin([1,0,0],0.);task.block('goal_live_obstacle_collision',0.)
    task.pause_inputs('inputs_invalid',.1)
    for i in range(155):task.tick([0,0,0],[0,0,0],0,True,'navigation',i*.1+.2)
    assert task.state=='FAILED' and task.failure_cause=='goal_live_obstacle_collision'
    task.begin([0,1,0],20.);assert task.failure_cause is None
