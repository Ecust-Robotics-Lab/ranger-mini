from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ws_nav/src/ranger_navigation'))
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.geometry import Body

def ring():
 a=np.linspace(-np.pi,np.pi,3600,endpoint=False)
 return np.column_stack((2*np.cos(a),2*np.sin(a),np.full(len(a),.4)))

@pytest.mark.parametrize('ttl',[.5,1.5,2.])
def test_recent_observations_bridge_missing_rays_and_expire(ttl):
 m=ObstacleMemory(control_memory=ttl,angular_guard=0)
 m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),10.)
 half=ring();half=half[half[:,0]<-.2]
 for stamp in [10.1,10.+ttl*.6,10.+ttl-.01]:
  m.update(half,np.eye(3),np.zeros(3),np.zeros(3),stamp)
  assert m.control_grid.free([.2,0.,0.],m.body,.055)
 m.update(half,np.eye(3),np.zeros(3),np.zeros(3),10.+ttl+.01)
 assert not m.control_grid.free([.2,0.,0.],m.body,.055)

def test_new_obstacle_revokes_remembered_free_shadow_immediately():
 m=ObstacleMemory(control_memory=.5,angular_guard=0)
 m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),10.)
 m.update(np.array([[.7,0,.4],[-2,0,.4]]),np.eye(3),np.zeros(3),np.zeros(3),10.1)
 tiny=Body(length=.01,width=.01)
 assert not m.control_grid.free([1.025,.025,0],tiny,0.)
 assert not m.covers([1.025,.025,0],tiny,0.)
 assert not m.update([],np.eye(3),np.zeros(3),np.zeros(3),10.2)
 assert not m.covers([0,0,0],m.body,0.)

def test_reset_discards_old_visibility():
 m=ObstacleMemory(control_memory=.5)
 m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),10.)
 m.reset()
 assert not m.covers([0,0,0],m.body,0.)


@pytest.mark.parametrize('shift',[(3,-2),(-4,5),(0,0),(90,0)])
def test_history_union_matches_world_cells_after_translation_and_body_rotation(shift):
    """Independent cell-index oracle, including disjoint history and expiry."""
    from ranger_navigation.planner import Grid
    rng=np.random.default_rng(37)
    m=ObstacleMemory(control_memory=.5,angular_guard=0)
    m.pose=np.array([.173,-.126,.83]);m.last_stamp=10.;m.error=None
    m.ranges=rng.uniform(.2,1.5,m.sectors)
    m._build_visibility_grid()
    current=m.control_grid.cells.copy();origin=m.visibility_pose.copy()
    old=np.where(rng.random((80,80))<.3,0,-1)
    observer=origin+np.array([shift[0]*.05,shift[1]*.05,0.])
    expected=current.copy()
    for y,x in np.ndindex(current.shape):
        # A cell at current origin + (x,y)*res is at these old indices.
        ox,oy=x-shift[0],y-shift[1]
        if 0<=ox<80 and 0<=oy<80 and old[oy,ox]==0:
            expected[y,x]=0
    m._visibility_history=[(9.8,observer,Grid(old,.05,(-2.,-2.,0.)))]
    m._build_visibility_grid(retain=True)
    np.testing.assert_array_equal(m.control_grid.cells,expected)
    np.testing.assert_array_equal(m._visible_grid.cells,expected)
    # Older obstacle memory may remain, but cannot extend the control TTL.
    m._visibility_history=[(9.4,observer,Grid(old,.05,(-2.,-2.,0.)))]
    m._build_visibility_grid(retain=True)
    np.testing.assert_array_equal(m.control_grid.cells,current)
    np.testing.assert_array_equal(m._visible_grid.cells,expected)
