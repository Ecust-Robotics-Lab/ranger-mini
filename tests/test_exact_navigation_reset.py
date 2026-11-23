"""Actual reset callback clears only navigation, protects active foreign goals."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'ws_nav/src/ranger_navigation'),str(ROOT/'ws_nav/src/ranger_localization_bridge')]
sys.path.insert(0,str(ROOT/'scripts'))
from navigation_health import navigation_compute_available
from ranger_navigation.coordinator import Coordinator
from ranger_navigation.task import Task
from rclpy.time import Time
from std_srvs.srv import Trigger
from rclpy.action import GoalResponse

def test_terminal_reset_revokes_generations_and_clears_both_published_paths():
    task=Task();task.begin([1,0,0],0.);task.invalidate('no_progress_timeout','FAILED')
    old_generation,old_history=task.generation,task.history_generation
    sent=[];pub=SimpleNamespace(publish=sent.append);worker=SimpleNamespace(nonce='old-compute',restart=Mock())
    pose=object();observations=object()
    node=SimpleNamespace(goal_handle=None,result_future=None,command=object(),task=task,worker=worker,
        path_pub=pub,local_pub=pub,route_pub=pub,expected_map='map',get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=10**9)),
        accepted_request_id=17,planning_failure={'reason':'blocked'},metrics={'old':True},estimate=pose,memory=observations)
    node.finish=lambda state:Coordinator.finish(node,state)
    response=Coordinator.reset_navigation(node,Trigger.Request(),Trigger.Response())
    assert response.success and json.loads(response.message)['previous_worker_nonce']=='old-compute'
    assert task.generation>old_generation and task.history_generation>old_history and task.goal is None
    assert node.command is None and node.accepted_request_id is None and not node.metrics
    assert node.estimate is pose and node.memory is observations
    assert len(sent)==3 and not sent[0].poses and not sent[1].poses and json.loads(sent[2].data)['segments']==[]
    worker.restart.assert_called_once()

def test_active_goal_is_never_aborted_by_the_reset_service():
    worker=SimpleNamespace(restart=Mock());node=SimpleNamespace(goal_handle=SimpleNamespace(is_active=True),worker=worker)
    response=Coordinator.reset_navigation(node,Trigger.Request(),Trigger.Response())
    assert not response.success and response.message=='active_goal_must_be_canceled_first'
    worker.restart.assert_not_called()

def test_goal_is_rejected_while_compute_is_resetting():
    node=SimpleNamespace(task=SimpleNamespace(state='RESETTING'))
    assert Coordinator.goal(node,object())==GoalResponse.REJECT

def test_managed_health_preserves_only_an_explicit_bounded_reset():
    status={'state':'RESETTING','worker':{'ready':False},'reset_started_steady':100.}
    assert navigation_compute_available(status,108.)
    assert navigation_compute_available(status,135.)
    assert not navigation_compute_available(status,135.001)
    assert not navigation_compute_available(status,99.9)
    for changed in [{'state':'EXECUTING'},{'reset_started_steady':None},
                    {'reset_started_steady':float('nan')},{'worker':{'ready':False,'error':'worker_exited'}}]:
        assert not navigation_compute_available({**status,**changed},108.)
    assert navigation_compute_available({'state':'IDLE','worker':{'ready':True}},108.)
