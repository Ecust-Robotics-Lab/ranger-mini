"""Independent geometric tests for the offline disconnection certificate."""
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from diagnose_exact_static_map import center_components


def test_point_connectivity_does_not_establish_body_connectivity():
    blocked = np.zeros((200, 300), dtype=bool)
    blocked[:, 145:155] = True
    blocked[78:122, 145:155] = False  # 44cm gap, narrower than 52cm body.
    distances = distance_transform_edt(~blocked)*.01
    points = [[.5, 1.], [2.5, 1.]]
    point, _ = center_components(blocked, distances, .01, [0, 0], points, 0.)
    body, _ = center_components(blocked, distances, .01, [0, 0], points, .26)
    assert point['connected']
    assert body['start_admitted'] and body['goal_admitted'] and not body['connected']


def test_sufficiently_wide_synthetic_gap_is_not_incorrectly_disconnected():
    blocked = np.zeros((200, 300), dtype=bool)
    blocked[:, 145:155] = True
    blocked[65:135, 145:155] = False  # 70cm gap.
    distances = distance_transform_edt(~blocked)*.01
    result, _ = center_components(blocked, distances, .01, [0, 0], [[.5,1.],[2.5,1.]], .275)
    assert result['connected']


def test_optimistic_mask_includes_exact_continuous_circle_centers():
    # Exact distances to complete forbidden cell squares, evaluated at random
    # continuous centers; this does not use the diagnostic distance transform.
    coarse = np.zeros((30, 40), dtype=bool)
    coarse[10:14, 18:20] = True; coarse[23, 4:8] = True
    fine = np.repeat(np.repeat(coarse, 5, axis=0), 5, axis=1)
    d = distance_transform_edt(~fine)*.01
    _, components = center_components(fine, d, .01, [0,0], [[.2,.2],[1.8,1.3]], .26)
    centers = (np.column_stack(np.nonzero(coarse)[::-1])+.5)*.05
    valid_count = 0
    for point in np.random.default_rng(19).uniform([0,0],[2.,1.5],(1000,2)):
        exact_distance = np.linalg.norm(np.maximum(np.abs(centers-point)-.025, 0.),axis=1).min()
        if exact_distance >= .26:
            x,y = np.floor(point/.01).astype(int)
            assert components[y,x] != 0
            valid_count += 1
    assert valid_count > 100
