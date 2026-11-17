"""Bind the user-confirmed camera front to geometry and chassis conversion."""
from pathlib import Path
import sys
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import load_geometry,rotation


def test_camera_optical_forward_matches_canonical_body_positive_x():
    _,Rbi,_=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml')
    cfg=yaml.safe_load((ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml').read_text())['/**']['ros__parameters']['extrin_calib']
    Ril=np.asarray(cfg['extrinsic_R']).reshape(3,3)
    Rcl=np.asarray(cfg['Rcl']).reshape(3,3)
    # Frontend uses Rci=Rcl*Ril.transpose(); camera optical +Z is view direction.
    forward=Rbi@Ril@Rcl.T@np.array([0.,0.,1.])
    assert forward[0]>.99
    assert np.linalg.norm(forward[1:])<.04


def test_imu_translation_respects_same_physical_front_distance():
    d=yaml.safe_load((ROOT/'config/fast_livo2/navigation_prep.yaml').read_text())
    _,Rbi,tbi=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml')
    ins=d['installation']
    assert np.isclose(ins['t_base_lidar_xyz'][0],d['body']['nominal_length_width_height_m'][0]/2-.26)
    assert np.allclose(tbi,np.array(ins['t_base_lidar_xyz'])+Rbi@np.array([.011,.02329,-.04412]))
    profile=yaml.safe_load((ROOT/'config/profiles/fast_livo2_frontend.yaml').read_text())['localization']
    assert np.allclose(profile['t_base_imu_xyz'],tbi)
    assert np.allclose(rotation(profile['t_base_imu_rpy']),Rbi)


def test_live_driver_explicitly_converts_opposite_native_axis():
    cfg=yaml.safe_load((ROOT/'config/base_mini_v2.yaml').read_text())['ranger_base_node']['ros__parameters']
    assert cfg['v2_sdk_front_opposite_base'] is True
    assert cfg['v2_feedback_steering_sign']==1.
