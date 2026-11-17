"""Coverage changes may reduce angular dilation, never invent missing rays."""
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body
from ranger_navigation.obstacles import ObstacleMemory


def visibility(sectors,guard,short_sector):
    memory=ObstacleMemory(Body(length=.01,width=.01),sectors=sectors,angular_guard=guard)
    memory.pose=np.zeros(3)
    memory.error=None
    memory.ranges=np.full(sectors,3.)
    memory.ranges[short_sector]=.1
    memory._build_visibility_grid()
    return memory


def test_neighbor_return_does_not_hide_a_cell_outside_its_sector():
    # This 5cm cell spans only azimuth sectors 90 and 91. A short return in
    # sector 92 formerly hid it through adjacent-sector dilation.
    point=[1.275,.025,0.]
    legacy=visibility(180,1,92)
    precise=visibility(180,0,92)
    assert not legacy.covers(point,legacy.body,0.)
    assert precise.covers(point,precise.body,0.)


def test_interior_sector_obstacle_cannot_hide_between_cell_corners():
    # At this range, a cell spans many fine sectors. Checking only its corners
    # could miss the short return at azimuth zero in the cell's interior.
    precise=visibility(720,0,360)
    assert not precise.covers([.525,.025,0.],precise.body,0.)


def test_empty_sector_remains_unknown_with_precise_coverage():
    precise=visibility(180,0,90)
    precise.ranges[90]=0.
    precise._build_visibility_grid()
    assert not precise.covers([.725,.025,0.],precise.body,0.)


def test_angular_wrap_checks_a_short_return_at_minus_pi():
    precise=visibility(360,0,0)
    assert not precise.covers([-.725,-.025,0.],precise.body,0.)
