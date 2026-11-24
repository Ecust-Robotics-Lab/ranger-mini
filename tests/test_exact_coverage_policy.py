"""Recorded guidance deadlock, bounded edge tolerance and protected body checks."""
from pathlib import Path
import sys,json
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body,densify,validate_sweep
from ranger_navigation.planner import Grid
from ranger_navigation.coverage_policy import CoveragePolicy
from ranger_navigation.guidance import observed_prefix
from ranger_navigation.obstacles import ObstacleMemory

def recorded():
    folder=ROOT/'tests/fixtures/coverage_margin'
    with np.load(folder/'original_request.npz',allow_pickle=False) as a:
        pose=a['pose'];observer=a['observer_pose'];grid=Grid(a['visibility_cells'],.05,(-2,-2,0.));points=a['points']
    with np.load(folder/'original_result.npz',allow_pickle=False) as a:path=a['segment_0']
    return pose,observer,grid,points,path

def test_real_frame_held_only_by_55mm_envelope_enters_new_prefix():
    pose,observer,grid,_,path=recorded();body=Body();path=densify(path,body,.025)
    old,diagnostic=observed_prefix(path,pose,np.zeros(3),observer,grid,body,CoveragePolicy(.055,.055,0,False))
    assert old is None and diagnostic['current']['unknown_protected_cells']==4
    current,diagnostic=observed_prefix(path,pose,np.zeros(3),observer,grid,body,CoveragePolicy())
    assert current is not None and len(current)>2 and diagnostic['checked_distance_m']>.10
    assert diagnostic['current']['accepted'] and diagnostic['current']['total_margin_m']==.025

def test_only_small_outer_unknown_count_is_allowed_without_repainting():
    body=Body(length=.74,width=.12);pose=[0,0,0]
    # Cover the whole body in a larger fine-grid fixture.
    cells=np.zeros((500,600));grid=Grid(cells,.002,(-.6,-.5,0.))
    def mark(count):
        cells=np.zeros_like(grid.cells)
        for i in range(count):cells[250+i,496]=-1 # x=.393, beyond protected .385
        return Grid(cells,.002,(-.6,-.5,0.))
    policy=CoveragePolicy()
    four=mark(4);before=four.cells.copy();report=four.coverage_report(pose,body,policy)
    assert report['accepted'] and report['unknown_edge_cells']==4 and report['unknown_protected_cells']==0
    np.testing.assert_array_equal(four.cells,before)
    assert not mark(9).covered(pose,body,policy)
    assert not four.covered(pose,body,CoveragePolicy(unknown_cell_budget=0))

def test_unknown_body_known_obstacle_and_outside_grid_cannot_use_edge_allowance():
    body=Body();cells=np.zeros((80,80));cells[40,40]=-1;grid=Grid(cells,.05,(-2,-2,0.))
    assert grid.coverage_report([0,0,0],body,CoveragePolicy())['reason']=='coverage_body_unobserved'
    cells[40,40]=100;grid=Grid(cells,.05,(-2,-2,0.))
    assert grid.coverage_report([0,0,0],body,CoveragePolicy())['reason']=='coverage_known_obstacle'
    assert not grid.covered([1.9,0,0],body,CoveragePolicy())
    assert not Grid(np.full((80,80),-1),.05,(-2,-2,0.)).covered([0,0,0],body,CoveragePolicy())

def test_guidance_cannot_skip_an_unobserved_current_body():
    cells=np.zeros((80,80));cells[40,33]=-1;grid=Grid(cells,.05,(-2,-2,0.))
    path=np.array([[.2,0.,0.],[.3,0.,0.]])
    assert grid.covered(path[0],Body(),CoveragePolicy())
    prefix,report=observed_prefix(path,np.zeros(3),np.zeros(3),np.zeros(3),grid,Body(),CoveragePolicy())
    assert prefix is None and not report['current']['accepted']

def test_partial_boundary_cells_share_quota_but_deep_unknown_never_does():
    body=Body();cells=np.zeros((80,80));cells[40,32]=-1;grid=Grid(cells,.05,(-2,-2,0))
    strict=grid.coverage_report([0,0,0],body,CoveragePolicy(allow_boundary_unknowns=False))
    allowed=grid.coverage_report([0,0,0],body,CoveragePolicy())
    assert not strict['accepted'] and strict['unknown_protected_cells']==1
    assert allowed['accepted'] and allowed['unknown_edge_cells']==1
    # Moving into that region places the entire unknown cell inside the body.
    assert not grid.covered([-.1,0,0],body,CoveragePolicy())
    cells[36:45,32]=-1;grid=Grid(cells,.05,(-2,-2,0))
    assert grid.coverage_report([0,0,0],body,CoveragePolicy())['reason']=='coverage_edge_budget_exceeded'

def test_latest_recorded_corner_boundary_no_longer_deadlocks():
    folder=ROOT/'tests/fixtures/coverage_speed'
    with np.load(folder/'recorded_boundary_fixture.npz') as data:
        grid=Grid(data['cells'],.05,(-2,-2,0));pose=data['pose']
    assert not grid.covered(pose,Body(),CoveragePolicy(allow_boundary_unknowns=False))
    report=grid.coverage_report(pose,Body(),CoveragePolicy())
    assert report['accepted'] and report['unknown_edge_cells']==4 and not report['unknown_protected_cells']

def test_final_sweep_uses_same_25mm_policy_and_still_rejects_physical_unknown():
    pose,observer,grid,points,path=recorded();body=Body();memory=ObstacleMemory(body)
    memory.set_visibility(observer,np.full(180,8.),grid.cells)
    # Recorded grid is in observer coordinates; give map poses to the coverage adapter.
    prefix,diag=observed_prefix(densify(path,body,.025),pose,np.zeros(3),observer,grid,body,CoveragePolicy())
    okay,_,reason=validate_sweep(prefix,points,body,coverage=memory.covers,margin=body.clearance+.04,coverage_margin=.025)
    assert okay,reason

def test_same_policy_under_nonzero_global_rotation():
    pose,observer,grid,_,path=recorded();body=Body();rotation=.7;c,s=np.cos(rotation),np.sin(rotation);r=np.array([[c,-s],[s,c]])
    shifted=path.copy();shifted[:,:2]=path[:,:2]@r.T+[4,-3];shifted[:,2]+=rotation
    start=np.r_[pose[:2]@r.T+[4,-3],pose[2]+rotation]
    correction=np.array([4.,-3.,rotation])
    a,_=observed_prefix(densify(path,body,.025),pose,np.zeros(3),observer,grid,body,CoveragePolicy())
    b,_=observed_prefix(densify(shifted,body,.025),start,correction,observer,grid,body,CoveragePolicy())
    assert a is not None and b is not None and len(a)==len(b)
