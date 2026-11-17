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
