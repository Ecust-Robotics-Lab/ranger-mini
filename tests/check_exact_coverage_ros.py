"""Installed coordinator/GPU/gate coverage integration; isolated mock plant, no CAN."""
import argparse,json,os,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tests.test_stage6_safety import World,rclpy,time

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-directory',type=Path,required=True)
    out=parser.parse_args().evidence_directory;out.mkdir(parents=True,exist_ok=True)
    assert not (out/'result.json').exists(),'preserve previous evidence'
    session=Path(tempfile.mkdtemp(prefix='coverage-ros-',dir=ROOT/'.runtime'))
    prefix=ROOT/'ws_nav/install_exact/ranger_navigation/lib/ranger_navigation'
    args=['--ros-args','-p','geometry_config:='+str(ROOT/'config/fast_livo2/navigation_prep.yaml'),
          '-p','coverage_protected_unknown_cell_budget:=2']
    processes=[];logs=[];checks=[];w=None
    result={'passed':False,'kind':'isolated_ros_mock_plant','hardware_started':False,'domain':os.environ['ROS_DOMAIN_ID']}
    try:
        for name,extra in [('motion_gate',[]),('exact_navigation',['-p','optimizer_python:='+str(ROOT/'.runtime/exact_mppi_venv/bin/python'),'-p','session_root:='+str(session),'-p','static_map_clearance:=0.01'])]:
            log=(out/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen([str(prefix/name),*args,*extra],stdout=log,stderr=subprocess.STDOUT))
        rclpy.init();w=World()
        w.wait(lambda:w.state.get('worker',{}).get('ready') and w.state.get('inputs_valid'),35)
        policy=w.state['worker']['coverage_policy']
        assert policy=={'total_margin':.025,'protected_margin':.015,'unknown_cell_budget':8,
                        'allow_boundary_unknowns':True,'protected_unknown_cell_budget':2},policy
        checks.append('installed_worker_reports_independent_protected_budget')
        w.wait(w.client.server_is_ready)
        assert w.state['control_limits']['max_speed']==.30
        h=w.goal(2.,0.);w.spin(.5);w.stopped_after(w.commands[0][0],.1)
        checks.append('unarmed_output_zero')
        w.arm(True)
        w.wait(lambda:any(np.linalg.norm(v)>.003 for _,v in w.commands[-5:]),10)
        checks.append('verified_gpu_candidate_passes_independent_gate')
        w.wait(lambda:any(np.linalg.norm(v[:2])>.23 for _,v in w.commands[-5:]),12)
        w.spin(1.)
        peak=max(float(np.linalg.norm(v[:2])) for _,v in w.commands)
        assert peak<=.30001
        checks.append('straight_cruise_exceeds_023_with_030_yaml_limit')
        w.cloud_on=False;started=time.monotonic();w.spin(.7);w.stopped_after(started,.35)
        checks.append('coverage_input_loss_stops_with_same_lease')
        w.cloud_on=True;f=h.cancel_goal_async();w.wait(f.done);started=time.monotonic();w.spin(.5);w.stopped_after(started,.15)
        checks.append('cancel_removes_previous_output')
        result.update(passed=True,coverage_policy=policy,peak_safe_command_m_s=peak,estimated_mock_pose=w.pose.tolist(),checks=checks,
                      end_to_end_ms=w.metrics,worker_session=str(session))
    except Exception as e:
        result.update(error=repr(e),checks=checks)
        raise
    finally:
        if w is not None:
            (out/'trace.json').write_text(json.dumps(w.trace))
            w.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for p in processes:
            if p.poll() is None:p.terminate()
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=5)
        for log in logs:log.close()
        result['owned_processes_stopped']=all(p.poll() is not None for p in processes)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    import numpy as np
    main()
