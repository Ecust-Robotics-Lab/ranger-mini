"""Cross-check JAX coverage decisions against independent grid SAT at rotated poses."""
from pathlib import Path
from types import SimpleNamespace
import sys,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
import jax.numpy as jnp
import pytest
from ranger_navigation.exact_controller import VisibilityCritics
from ranger_navigation.geometry import Body
from ranger_navigation.coverage_policy import CoveragePolicy
from ranger_navigation.planner import Grid

class NoOtherCosts:
 def evalTrajectoriesScores(self,data):return jnp.zeros(data.trajectories.x.shape[0]),jnp.array(False),None

def evaluate(cells,poses,observer,policy=CoveragePolicy()):
 c,s=np.cos(observer[2]),np.sin(observer[2]);r=np.array([[c,-s],[s,c]])
 world=np.asarray(poses).copy();world[:,:2]=world[:,:2]@r.T+observer[:2];world[:,2]+=observer[2]
 data=SimpleNamespace(trajectories=SimpleNamespace(x=jnp.asarray(world[:,0,None]),y=jnp.asarray(world[:,1,None]),yaws=jnp.asarray(world[:,2,None])),
  state=SimpleNamespace(coverage_observer=jnp.asarray(observer),coverage_cells=jnp.asarray(cells),coverage_enabled=jnp.array(True)))
 costs,_,_=VisibilityCritics(NoOtherCosts(),Body(),coverage_policy=policy).evalTrajectoriesScores(data)
 return np.asarray(costs)

@pytest.mark.parametrize('boundary',[True,False])
def test_jax_and_cpu_agree_for_sparse_edge_and_protected_unknowns(boundary):
 cells=np.zeros((80,80),dtype=np.int16);cells[38:42,48]=-1
 rng=np.random.default_rng(53);poses=np.column_stack((rng.uniform(-.7,.7,(64,2)),rng.uniform(-np.pi,np.pi,64)))
 poses=np.vstack(([.0123,0,0],[1.8,0,0],poses));grid=Grid(cells,.05,(-2,-2,0))
 policy=CoveragePolicy(allow_boundary_unknowns=boundary)
 costs=evaluate(cells,poses,np.array([1.1,-.7,.6]),policy)
 expected=np.array([grid.covered(p,Body(),policy) for p in poses])
 np.testing.assert_array_equal(costs<1e6,expected)
 assert expected[0] and not expected[1]
 assert grid.coverage_report(poses[0],Body(),CoveragePolicy())['unknown_edge_cells']==4

def test_real_occupied_edge_cell_is_never_an_unknown_allowance():
 cells=np.zeros((80,80),dtype=np.int16);cells[40,48]=100
 costs=evaluate(cells,np.array([[.0123,0,0]]),np.zeros(3))
 assert costs[0]>=1e6


@pytest.mark.parametrize(('protected_budget', 'edge_budget', 'accepted'), [
    (0, 16, False),
    (1, 16, False),
    (2, 7, True),
    (2, 6, False),
])
def test_two_interior_and_seven_edge_cells_use_separate_budgets(protected_budget, edge_budget, accepted):
    cells = np.zeros((80, 80), dtype=np.int16)
    cells[40, 40:42] = -1
    cells[34, 34:41] = -1
    pose = np.zeros(3)
    policy = CoveragePolicy(total_margin=.015, unknown_cell_budget=edge_budget,
                            protected_unknown_cell_budget=protected_budget)
    report = Grid(cells, .05, (-2, -2, 0)).coverage_report(pose, Body(), policy)
    assert (report['unknown_protected_cells'], report['unknown_edge_cells']) == (2, 7)
    assert report['accepted'] == accepted
    assert bool(evaluate(cells, [pose], pose, policy)[0] < 1e6) == accepted
    cells[40, 40] = 100
    assert not Grid(cells, .05, (-2, -2, 0)).covered(pose, Body(), policy)
    assert evaluate(cells, [pose], pose, policy)[0] >= 1e6
