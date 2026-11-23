"""A retained sequence must be projected from feedback and fully revalidated."""
from pathlib import Path
import sys,numpy as np,pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ws_nav/src/ranger_navigation'))
from ranger_navigation.exact_controller import ExactController
from ranger_navigation.models import Mode,feasible
from ranger_navigation.geometry import validate_sweep


@pytest.fixture(scope='module')
def controller():return ExactController(samples=128)


@pytest.mark.parametrize('mode,goal',[(Mode.DRIVE,[-.6,0,0]),(Mode.PARALLEL,[0,.6,0]),(Mode.SPIN,[0,0,.8]),(Mode.SPIN,[0,0,-.8])])
def test_warm_sequence_preserves_signed_modes_and_feedback_acceleration(controller,mode,goal):
    c=controller;c.reset();path=np.array([[0.,0.,0.],goal]);first=c.compute(np.zeros(3),np.zeros(3),path,mode,[],advance_history=False)
    speed=first['command'];pose=first['trajectory'][1]
    result=c.compute(pose,speed,path,mode,[],warm_start=first['controls'],advance_history=False)
    assert all(feasible(v,mode,c.limits) for v in result['controls'])
    difference=np.diff(np.vstack((speed,result['controls'])),axis=0)
    assert np.linalg.norm(difference[:,:2],axis=1).max()<=.01501
    assert np.abs(difference[:,2]).max()<=.02501
    assert validate_sweep(result['trajectory'],[],c.body)[0]
    np.testing.assert_allclose(result['trajectory'][0],pose)
    assert np.dot(result['command'],speed)>0
