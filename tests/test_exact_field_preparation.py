"""Field preparation and narrow-space contracts; no devices or ROS graph."""
import copy
import json
from pathlib import Path
import sys
import tempfile

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'ws_nav/src/ranger_navigation')]
from prepare_exact_field_test import read_route, check_leg
from ranger_navigation.geometry import Body, validate_sweep
from ranger_navigation.models import Limits
from ranger_navigation.planner import Grid, Planner


def corridor(unknown=False):
    cells = np.full((80, 140), -1 if unknown else 100, dtype=np.int16)
    cells[33:46, :] = 0  # 65cm actual raster width; body yaw is zero.
    return Grid(cells, .05, (-3.5, -2., 0.))


@pytest.mark.parametrize('unknown', [False, True])
def test_narrow_corridor_passes_without_shrinking_body_or_freeing_unknown(unknown):
    grid = corridor(unknown)
    start, goal = [-1., -.025, 0.], [1., -.025, 0.]
    regular = check_leg(grid, Body(), Limits(), start, goal, budget=.5)
    assert not regular['accepted']
    body = Body(clearance=.03)
    result = check_leg(grid, body, Limits(), start, goal, static_reserve=.01, budget=2.)
    assert result['accepted'], result
    assert body.length == .74 and body.width == .52
    assert validate_sweep(result['path'], [], body, grid=grid, margin=.07, map_margin=.04)[0]
    # The same width cannot admit a 90deg turn of the full 74cm body.
    turn = check_leg(grid, body, Limits(), start, [-1., -.025, np.pi/2],
                     static_reserve=.01, budget=.1)
    assert not turn['accepted']


def test_map_reserve_does_not_reduce_voxel_uncertainty_or_observed_space_checks():
    body = Body(clearance=.03)
    point = [[0., .26+.084]]
    assert not validate_sweep([[0, 0, 0]], point, body, margin=.07, map_margin=.04)[0]
    assert not validate_sweep([[0, 0, 0]], [], body, margin=.07, map_margin=.04,
                              coverage=lambda *args: False)[0]
    grid = corridor(True)
    grid.cells[39:41, 65:75] = -1
    grid = Grid(grid.cells, grid.resolution, grid.origin)
    assert not check_leg(grid, body, Limits(), [-1., -.025, 0.], [1., -.025, 0.],
                          static_reserve=.01, budget=.2)['accepted']


def test_narrow_passage_prefers_fixed_steering_translation():
    result=check_leg(corridor(),Body(clearance=.03),Limits(),[-1.,-.025,0.],[1.,-.025,0.],
                     static_reserve=.01,prefer_parallel_in_narrow=True,budget=2.)
    assert result['accepted']
    assert result['segments'] and all(s['mode']=='PARALLEL' for s in result['segments'])


@pytest.mark.parametrize('reserve', [-.001, float('nan'), .11])
def test_invalid_reserve_rejected(reserve):
    with pytest.raises(ValueError):
        Planner(static_reserve=reserve)


def test_wrong_map_and_nonfinite_or_open_return_route_rejected():
    route = yaml.safe_load((ROOT/'config/exact_mppi/office_corridor_route.yaml').read_text())
    manifest = {k: route[k] for k in ('map_uuid', 'source_manifest_sha256')}
    cases = []
    for key, value in [('frame', 'odom'), ('map_uuid', 'wrong'), ('source_manifest_sha256', '0'*64)]:
        r = copy.deepcopy(route); r[key] = value; cases.append(r)
    r = copy.deepcopy(route); r['waypoints'][0]['pose'][0] = float('nan'); cases.append(r)
    r = copy.deepcopy(route); r['waypoints'][-1]['pose'][0] += 1.; cases.append(r)
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp)/'route.json'
        path.write_text(json.dumps(route)); assert read_route(path, manifest)['human_confirmed'] is False
        for bad in cases:
            path.write_text(json.dumps(bad))
            with pytest.raises(ValueError):
                read_route(path, manifest)


def test_navigation_recording_topics_are_optional_and_do_not_become_required_inputs():
    import importlib.util
    spec = importlib.util.spec_from_file_location('field_online_session', ROOT/'scripts/fast_livo2/localizer/online_session.py')
    online = importlib.util.module_from_spec(spec); spec.loader.exec_module(online)
    args = online.parser().parse_args(['--public-tf', '--record-navigation'])
    topics = online.record_topics(args)
    assert '/base/safe_cmd_vel' in topics and '/motion_state' in topics
    assert '/navigate_to_pose/_action/status' in topics
    assert '/base/safe_cmd_vel' not in online.mapping.required_topics(args)
    assert len(topics) == len(set(topics))
    from record_session import recorder_command
    command = recorder_command(Path('/tmp/unused'), topics)
    assert '--include-hidden-topics' in command
    assert '--all' not in command and '-a' not in command
    args.record_navigation = False
    assert '/base/safe_cmd_vel' not in online.record_topics(args)
    assert '--include-hidden-topics' not in recorder_command(Path('/tmp/unused'), online.record_topics(args))


@pytest.mark.parametrize('profile,clearance,reserve,visible', [('standard',.04,.0,.10), ('narrow',.03,.01,.04)])
@pytest.mark.parametrize('requested_speed',[0.,.30])
def test_launch_profile_reaches_both_processes_with_same_physical_limits(monkeypatch, profile, clearance, reserve, visible,requested_speed):
    import importlib.util
    launch = pytest.importorskip('launch')
    spec = importlib.util.spec_from_file_location('field_launch', ROOT/'ws_nav/src/ranger_navigation/launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = launch.LaunchContext()
    context.launch_configurations.update(execute='true', control_authorized='true', project_root=str(ROOT),
        map_yaml=str(ROOT/'outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final/map.yaml'),
        optimizer_python=str(ROOT/'.runtime/exact_mppi_venv/bin/python3'), wheel_topic='/fast_livo/navigation/wheel/odom',
        use_sim_time='true', low_speed='true', start_map_server='false', clearance_profile=profile)
    nodes = module.prepare(context)  # Node factory is mocked; no ROS process starts.
    configured_coverage = yaml.safe_load(
        (ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())['coverage']
    if requested_speed:
        context.launch_configurations['max_speed']=str(requested_speed)
        nodes=module.prepare(context)
    assert len(nodes) == 2
    for node in nodes:
        common = node['parameters'][0]
        assert common['clearance'] == clearance
        assert common['max_speed'] == (requested_speed or .1) and common['max_yaw_rate'] == .2
        assert common['geometry_config'] == str(ROOT/'config/fast_livo2/navigation_prep.yaml')
        for key, value in configured_coverage.items():
            assert common[f'coverage_{key}'] == value
    assert nodes[0]['parameters'][1]['static_map_reserve'] == reserve
    assert nodes[0]['parameters'][1]['static_map_clearance']==.01
    assert nodes[0]['parameters'][1]['visibility_reserve'] == visible
    assert nodes[0]['parameters'][1]['prefer_parallel_in_narrow'] == (profile=='narrow')


def test_worker_receives_explicit_narrow_profile_without_starting_process(monkeypatch, tmp_path):
    from ranger_navigation.worker import WorkerClient
    from ranger_navigation.coverage_policy import CoveragePolicy
    import ranger_navigation.worker as worker
    seen = []
    class MockProcess:
        def __init__(self, argv, **kwargs): seen.append(argv)
        def poll(self): return 0
    monkeypatch.setattr(worker.subprocess, 'Popen', MockProcess)
    client = WorkerClient(tmp_path, sys.executable, ROOT/'config/fast_livo2/navigation_prep.yaml',
                          clearance=.03, static_reserve=.01, visibility_reserve=.04,prefer_parallel_in_narrow=True,
                          coverage_policy=CoveragePolicy(.035,.02,4,False,protected_unknown_cell_budget=2))
    client.close()
    cmd = seen[0]
    for key, value in [('--clearance','0.03'), ('--static-reserve','0.01'), ('--visibility-reserve','0.04'),
                       ('--coverage-margin','0.035'),('--coverage-protected-margin','0.02'),('--coverage-unknown-budget','4'),
                       ('--coverage-protected-unknown-budget','2'),
                       ('--coverage-boundary-unknowns','false')]:
        assert cmd[cmd.index(key)+1] == value
    assert '--prefer-parallel-in-narrow' in cmd


@pytest.mark.parametrize('coverage,valid',[
    ({'total_margin':.035,'protected_margin':.02,'unknown_cell_budget':4,'allow_boundary_unknowns':False,
      'protected_unknown_cell_budget':2},True),
    ({'total_margin':.025,'protected_margin':.015,'unknown_cell_budget':0},True),
    ({'total_margin':.025,'protected_margin':.015,'unknown_cell_budget':2.5},False),
    ({'total_margin':.025,'protected_margin':.015,'unknown_cell_budget':17},False),
    ({'total_margin':.025,'protected_margin':.015,'unknown_cell_budget':-1},False),
    ({'total_margin':.01,'protected_margin':.015,'unknown_cell_budget':8},False),
    ({'total_margin':.025,'protected_margin':.015,'unknown_cell_budget':8,'allow_boundary_unknowns':'yes'},False),
    ({'protected_unknown_cell_budget':-1},False),
    ({'protected_unknown_cell_budget':17},False),
    ({'protected_unknown_cell_budget':True},False),
])
def test_editable_yaml_reaches_both_nodes_or_rejects_before_start(monkeypatch,tmp_path,coverage,valid):
    import importlib.util
    launch=pytest.importorskip('launch')
    spec=importlib.util.spec_from_file_location('yaml_launch',ROOT/'ws_nav/src/ranger_navigation/launch/navigation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    declared=next(action for action in module.generate_launch_description().entities
                  if getattr(action,'name',None)=='low_speed')
    assert ''.join(part.perform(launch.LaunchContext()) for part in declared.default_value)=='false'
    nodes=[]
    def node(**kwargs):nodes.append(kwargs);return kwargs
    monkeypatch.setattr(module,'Node',node)
    config=yaml.safe_load((ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())
    config['coverage']=coverage;path=tmp_path/'navigation.yaml';path.write_text(yaml.safe_dump(config))
    config['motion'].update(max_speed=.24,max_yaw_rate=.35)
    path.write_text(yaml.safe_dump(config))
    context=launch.LaunchContext()
    context.launch_configurations.update(execute='true',control_authorized='true',project_root=str(ROOT),
        navigation_config=str(path),map_yaml=str(ROOT/'outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final/map.yaml'),
        optimizer_python=str(ROOT/'.runtime/exact_mppi_venv/bin/python3'),wheel_topic='/fast_livo/navigation/wheel/odom',
        use_sim_time='true',start_map_server='false',clearance_profile='standard')
    if not valid:
        with pytest.raises(ValueError,match='coverage'):module.prepare(context)
        assert not nodes
        return
    assert len(module.prepare(context))==2
    for node in nodes:
        params=node['parameters'][0]
        assert params['coverage_total_margin']==coverage['total_margin']
        assert params['coverage_protected_margin']==coverage['protected_margin']
        assert params['coverage_unknown_cell_budget']==coverage['unknown_cell_budget']
        assert params['coverage_protected_unknown_cell_budget']==coverage.get('protected_unknown_cell_budget',0)
        assert params['max_speed']==.24 and params['max_yaw_rate']==.35
        assert params['coverage_allow_boundary_unknowns']==coverage.get('allow_boundary_unknowns',True)


@pytest.mark.parametrize('mode,error', [(0,[0.,0.,.02]), (0,[.01,0.,0.]), (1,[0.,0.,.02])])
def test_mode_entry_replans_from_actual_stopped_pose(mode, error):
    from ranger_navigation.task import Task
    from ranger_navigation.models import Mode
    from ranger_navigation.planner import Segment
    task=Task(segment_yaw_tolerance=.03);task.begin([1.,0.,0.],0.)
    task.plan([Segment(Mode(mode),np.array([[0.,0.,0.],[1.,0.,0.]]))],task.generation,0.)
    assert task.tick(np.array(error),np.zeros(3),mode,True,'navigation',1.) is None
    assert task.state=='PLANNING' and task.reason=='segment_start_pose_replan'


def test_spin_uses_actual_center_without_replanning_terminal_position_error_forever():
    from ranger_navigation.task import Task
    from ranger_navigation.models import Mode
    from ranger_navigation.planner import Segment
    task=Task(segment_yaw_tolerance=.03);task.begin([1.,0.,0.],0.)
    segment=Segment(Mode.SPIN,np.array([[1.,0.,.7],[1.,0.,0.]]))
    task.plan([segment],task.generation,0.)
    pose=np.array([.96,0.,.7])
    task.tick(pose,np.zeros(3),Mode.SPIN,True,'navigation',.1)
    task.tick(pose,np.zeros(3),Mode.SPIN,True,'navigation',.7)
    assert task.tick(pose,np.zeros(3),Mode.SPIN,True,'navigation',.8) is segment
