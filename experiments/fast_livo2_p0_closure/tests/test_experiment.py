"""Offline numerical/lifecycle contract tests. No ROS node or device is created."""
import hashlib
import ast
import json
import math
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import analysis as a
import suite


def p(x=0., y=0., angle=0.):
    return {'xyz': [x, y, 0.], 'xyzw': [0., 0., math.sin(angle/2), math.cos(angle/2)]}


class NumericalTests(unittest.TestCase):
    def setUp(self):
        self.cfg = a.load(HERE/'config.json')

    def test_yaw_wrap(self):
        start = {'valid': True, 'position_xyz': [0, 0, 0], 'yaw_rad': math.radians(179)}
        end = {'valid': True, 'position_xyz': [.1, 0, 0], 'yaw_rad': math.radians(-179)}
        m = a.closure(start, end, 20., None)
        self.assertAlmostEqual(m['yaw_error_deg'], 2.)
        self.assertAlmostEqual(m['estimated_relative_drift_rate_percent'], .5)
        self.assertIsNone(m['relative_drift_rate_percent'])
        self.assertAlmostEqual(a.closure(start, end, 20., 40.)['relative_drift_rate_percent'], .25)

    def test_nonzero_base_transform(self):
        cal = {'installation': {'t_base_imu_rpy': [.1, .3, -.2], 't_base_imu_xyz': [.12, .03, .55]}}
        ext = a.base_extrinsic(cal)
        truth = a.matrix(p(2., 3., .7))
        np.testing.assert_allclose(truth @ ext @ np.linalg.inv(ext), truth, atol=1e-12)
        self.assertAlmostEqual(a.yaw(truth @ ext @ np.linalg.inv(ext)), .7)

    def test_nonfinite_pose_rejected(self):
        with self.assertRaises(ValueError):
            a.matrix(p(float('nan')))

    def test_final_graph_reconstruction_has_no_future_reference(self):
        rows = [{'stamp_ns': n, 'pose': p(x)} for n, x in [(9, 3), (10, 3), (20, 5)]]
        graph = {'keyframes': [{'id': 0, 'stamp_ns': 10, 'U_local_imu': p(2), 'X_map_imu': p(1)}]}
        result = a.optimized_rows(rows, graph)
        self.assertEqual([r['stamp_ns'] for r in result], [10, 20])
        self.assertAlmostEqual(result[1]['T_imu'][0, 3], 4.)

    def test_first_invalid_output_not_promoted_by_heartbeat(self):
        row = {'kind': 'online', 'session_id': 's', 'stamp_ns': 10, 'scan_id': 1,
               'pose_valid': False, 'state': 2, 'sample_steady_ns': 20, 'sample_age_s': .01}
        self.assertEqual(a.online_rows([row, {**row, 'pose_valid': True, 'state': 1}], self.cfg), [])

    def test_stale_and_future_outputs_rejected(self):
        base = {'kind': 'online', 'session_id': 's', 'pose_valid': True, 'state': 1,
                'sample_steady_ns': 20}
        rows = [{**base, 'stamp_ns': i, 'scan_id': i, 'sample_age_s': age}
                for i, age in enumerate([.6, -.2, .1])]
        self.assertEqual(len(a.online_rows(rows, self.cfg)), 1)

    def test_window_uses_availability_deadline(self):
        rows = [{'sample_steady_ns': i*10**8, 'received_steady_ns': 5*10**9, 'pose': p()}
                for i in range(21)]
        self.assertFalse(a.window(rows, [0, 2*10**9], np.eye(4), self.cfg, True)['valid'])
        self.assertTrue(a.window(rows, [0, 2*10**9], np.eye(4), self.cfg)['valid'])

    def test_window_gap_and_step_flag(self):
        rows = [{'sample_steady_ns': i*10**8, 'pose': p(.1 if i >= 10 else 0)} for i in range(21)]
        value = a.window(rows, [0, 2*10**9], np.eye(4), self.cfg)
        self.assertTrue(value['jump_detected'])
        self.assertTrue(value['valid'])
        sparse = [rows[0], rows[-1]]
        self.assertFalse(a.window(sparse, [0, 2*10**9], np.eye(4), self.cfg)['valid'])

    def test_path_is_length_not_endpoint_displacement(self):
        rows = [{'sample_steady_ns': i, 'pose': p(x)} for i, x in enumerate([0, 1, 2, 1, 0])]
        self.assertAlmostEqual(a.path_length(rows, 0, 4, np.eye(4), .02), 4.)

    def test_mapping_plan_uses_latency_camera_and_no_base(self):
        # Module import/configuration and dry-run planning only: no rclpy.init.
        import mapping_entry
        mapping_entry.configure()
        m = mapping_entry.mapping
        cfg = m.resolve(ROOT)
        folder = ROOT/'.runtime/validation/p0_experiment_plan_only'
        m.environment(cfg, folder)
        args = m.parser().parse_args(['--visualization', 'none'])
        items = m.components(cfg, args, folder)
        camera = next(c for c in items if c.name == 'camera')
        self.assertTrue(any('latency_install' in v for v in camera.argv))
        self.assertFalse(any('ranger_base' in str(c.argv) or 'cmd_vel' in str(c.argv) for c in items))
        self.assertIn('closure_observer', next(c for c in items if c.name == 'backend').depends)
        frontend = next(c for c in items if c.name == 'frontend')
        self.assertIn('clock_ready', frontend.start_requires)
        self.assertNotIn('clock_ready', frontend.preflight_requires)
        self.assertIn('use_sim_time:=true', frontend.argv)


class EndToEndTests(unittest.TestCase):
    def fixture(self, trial):
        cfg = a.load(HERE/'config.json')
        capture = trial/'capture'
        (capture/'map').mkdir(parents=True)
        (capture/'raw').mkdir()
        a.save(trial/'config.json', cfg)
        a.save(trial/'calibration.json', {'installation': {'t_base_imu_xyz': [0, 0, 0], 't_base_imu_rpy': [0, 0, 0]}})
        a.save(trial/'trial.json', {'route_label': 'synthetic_only'})
        origin, sensor = 10**12, 1590190000000000000
        a.save(trial/'events.json', {'start_window': [origin, origin+3*10**9],
                                    'route_start_ns': origin+3*10**9, 'returned_ns': origin+12*10**9})
        a.save(capture/'closure_observer.json', {'anchor': [sensor, origin]})
        graph = {'session_id': 's', 'graph_version': 2, 'keyframes': [
            {'id': 0, 'stamp_ns': sensor, 'U_local_imu': p(), 'X_map_imu': p()},
            {'id': 1, 'stamp_ns': sensor+12*10**9, 'U_local_imu': p(.1, 0, .02), 'X_map_imu': p()}]}
        a.save(capture/'map/manifest.json', graph)
        a.save(capture/'map/COMMITTED', {'manifest_sha256': hashlib.sha256((capture/'map/manifest.json').read_bytes()).hexdigest()})
        a.save(capture/'mapping_result.json', {'success': True})
        rows = []
        from rclpy.serialization import serialize_message
        from nav_msgs.msg import Odometry
        with sqlite3.connect(capture/'raw/test.db3') as db:
            db.execute('CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT)')
            db.execute('CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
            db.execute('INSERT INTO topics VALUES (1, ?)', ('/fast_livo/odometry',))
            for i in range(281):
                sec = i/10
                x = 0 if sec <= 3 else (sec-3)*2 if sec < 7.5 else (12-sec)*2+.1 if sec < 12 else .1
                q = p(x, 0, .02 if sec >= 12 else 0)
                stamp, steady = sensor+i*10**8, origin+i*10**8
                row = {'kind': 'frontend', 'stamp_ns': stamp, 'sample_steady_ns': steady,
                       'received_steady_ns': steady+20_000_000, 'pose': q}
                rows.append(row)
                online_pose = p() if sec >= 16 else q
                rows.append({**row, 'kind': 'online', 'pose': online_pose, 'session_id': 's',
                    'scan_id': i, 'graph_version': 1 if sec < 16 else 2, 'pose_valid': True,
                    'state': 1, 'sample_age_s': .02})
                m = Odometry()
                m.header.frame_id, m.child_frame_id = 'fast_livo_local', 'fast_livo_imu'
                m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
                m.pose.pose.position.x = float(x)
                m.pose.pose.orientation.z, m.pose.pose.orientation.w = q['xyzw'][2:]
                db.execute('INSERT INTO messages VALUES (?,?,?,?)', (i+1, 1, stamp, serialize_message(m)))
        (capture/'closure_samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        return capture

    def test_complete_three_way_analysis_and_preserved_reanalysis(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial = Path(tmp)
            self.fixture(trial)
            result, out = a.analyze(trial)
            self.assertEqual(result['status'], 'PASS_DATA')
            raw = result['trajectories']['frontend']['metrics']['settled']
            hist = result['trajectories']['optimized_history']['metrics']['settled']
            live = result['trajectories']['online_backend']['metrics']
            self.assertAlmostEqual(raw['position_error_m'], .1)
            self.assertAlmostEqual(hist['position_error_m'], 0, places=10)
            self.assertAlmostEqual(live['immediate']['position_error_m'], .1)
            self.assertAlmostEqual(live['settled']['position_error_m'], 0)
            self.assertIsNone(raw['relative_drift_rate_percent'])
            self.assertTrue((out/'trajectories.png').exists())
            second, other = a.analyze(trial, measured_length=20.)
            self.assertNotEqual(out, other)
            self.assertTrue((out/'metrics.json').exists())
            self.assertAlmostEqual(second['trajectories']['frontend']['metrics']['settled']['relative_drift_rate_percent'], .5)

    def test_checksum_corruption_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial = Path(tmp)
            cap = self.fixture(trial)
            (cap/'map/manifest.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'checksum'):
                a.analyze(trial)

    def test_cross_session_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial = Path(tmp)
            cap = self.fixture(trial)
            with (cap/'closure_samples.jsonl').open('a') as handle:
                handle.write(json.dumps({'kind': 'online', 'session_id': 'foreign'})+'\n')
            with self.assertRaisesRegex(ValueError, 'session'):
                a.analyze(trial)

    def test_missing_return_never_produces_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial = Path(tmp)
            a.save(trial/'events.json', {'route_start_ns': 1})
            with self.assertRaisesRegex(ValueError, 'return'):
                a.analyze(trial)

    def test_prestart_discovery_heartbeat_is_not_measurement_reset(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial = Path(tmp)
            cap = self.fixture(trial)
            path = cap/'closure_samples.jsonl'
            row = {'kind': 'online', 'session_id': 'discovery-only', 'stamp_ns': 0,
                   'scan_id': 0, 'state': 0, 'pose_valid': False, 'sample_age_s': None,
                   'sample_steady_ns': None, 'received_steady_ns': 10**12-10**9, 'pose': p()}
            path.write_text(json.dumps(row)+'\n'+path.read_text())
            result, _ = a.analyze(trial)
            self.assertEqual(result['status'], 'PASS_DATA')
            self.assertEqual(result['ignored_prestart_session_ids'], ['discovery-only'])
            self.assertEqual(result['measurement_session_ids'], ['s'])

    def test_invalid_foreign_session_during_measurement_still_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial = Path(tmp)
            cap = self.fixture(trial)
            with (cap/'closure_samples.jsonl').open('a') as handle:
                handle.write(json.dumps({'kind': 'online', 'session_id': 'foreign', 'pose_valid': False,
                                         'received_steady_ns': 10**12+10**9})+'\n')
            with self.assertRaisesRegex(ValueError, 'session'):
                a.analyze(trial)


class LifecycleTests(unittest.TestCase):
    def exercise(self, abort):
        # Real owned subprocess and signals, with fake files only. No DDS,
        # sensors, CAN or robot executables are launched.
        script = '''import json,signal,sys,time
from pathlib import Path
folder=Path(sys.argv[1]);folder.mkdir()
stopped=False
def stop(*args):
 global stopped
 stopped=True
signal.signal(signal.SIGTERM,stop)
checks={k:True for k in ('frontend','backend_ready','capture_recording','clock_ok')}
(folder/'mapping_health.json').write_text(json.dumps({'checks':checks}))
(folder/'closure_observer.json').write_text(json.dumps({'anchor':[1,1],'counts':{'frontend':5,'online':5},'monotonic_ns':time.monotonic_ns()}))
while not stopped:time.sleep(.01)
(folder/'fake_finalized').write_text('owned subprocess stopped cleanly')
'''
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            a.save(run/'calibration.json', {})
            cfg = a.load(HERE/'config.json')
            cfg.update(warmup_seconds=.01, start_window_seconds=.01, end_hold_seconds=.01,
                       startup_timeout_seconds=3, min_free_gb=0)
            progress = {'trials': {}}
            answers = ['短', '', KeyboardInterrupt('synthetic abort')] if abort else ['短', '', '']
            def fake_analysis(trial):
                self.assertTrue((trial/'capture/fake_finalized').exists())
                events = a.load(trial/'events.json')
                self.assertLess(events['route_start_ns'], events['returned_ns'])
                return {'status': 'PASS_DATA', 'trajectories': {}}, trial/'fake_result'
            with mock.patch.object(suite, 'verify_frozen'), mock.patch.object(suite, 'prompt', side_effect=answers), \
                    mock.patch.object(suite, 'command', side_effect=lambda c, t: [sys.executable, '-c', script, str(t/'capture')]), \
                    mock.patch.object(suite.analysis, 'analyze', side_effect=fake_analysis):
                if abort:
                    with self.assertRaises(KeyboardInterrupt):
                        suite.run_trial(run, 1, cfg, progress)
                else:
                    suite.run_trial(run, 1, cfg, progress)
            trial = Path(progress['trials']['1']['directory'])
            self.assertTrue((trial/'capture/fake_finalized').exists())
            self.assertFalse(suite.same_process(a.load(trial/'mapping_process.json')))
            self.assertEqual(progress['trials']['1']['status'], 'FAILED' if abort else 'COMPLETE')
            self.assertEqual((trial/'failure.json').exists(), abort)

    def test_normal_owned_finalization(self):
        self.exercise(False)

    def test_abort_preserves_evidence_and_stops_owned_process(self):
        self.exercise(True)


class ProvenanceTests(unittest.TestCase):
    def fixture(self, run):
        prefix = str(HERE.relative_to(ROOT))+'/'
        source = (HERE/'suite.py').read_text()
        tree = ast.parse(source)
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == 'verify_frozen':
                node.body = [ast.Pass()]
        old = ast.unparse(tree)
        (run/'experiment_sources').mkdir()
        (run/'experiment_sources/suite.py').write_text(old)
        frozen = {prefix+'suite.py': hashlib.sha256(old.encode()).hexdigest(),
                  prefix+'analysis.py': 'old-evaluator', 'backend': 'same-backend'}
        current = {**frozen, prefix+'suite.py': hashlib.sha256(source.encode()).hexdigest(),
                   prefix+'analysis.py': 'new-evaluator'}
        a.save(run/'source_hashes.json', frozen)
        return frozen, current

    def test_offline_analysis_revision_retains_original_acquisition_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            frozen, current = self.fixture(run)
            with mock.patch.object(suite, 'hashes', return_value=current):
                suite.verify_frozen(run)
            self.assertEqual(a.load(run/'source_hashes.json'), frozen)
            self.assertEqual(len(list((run/'analysis_revisions').glob('*.json'))), 1)

    def test_backend_change_cannot_enter_existing_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            _, current = self.fixture(run)
            current['backend'] = 'changed-backend'
            with mock.patch.object(suite, 'hashes', return_value=current), self.assertRaises(RuntimeError):
                suite.verify_frozen(run)

    def test_changed_acquisition_runner_cannot_enter_existing_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            frozen, current = self.fixture(run)
            old = run/'experiment_sources/suite.py'
            old.write_text(old.read_text()+'\nacquisition_changed = True\n')
            frozen[str(HERE.relative_to(ROOT))+'/suite.py'] = hashlib.sha256(old.read_bytes()).hexdigest()
            a.save(run/'source_hashes.json', frozen)
            with mock.patch.object(suite, 'hashes', return_value=current), self.assertRaises(RuntimeError):
                suite.verify_frozen(run)

    def test_changed_per_trial_command_cannot_enter_existing_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            frozen, current = self.fixture(run)
            old = run/'experiment_sources/suite.py'
            old.write_text(old.read_text().replace('def command(cfg, trial):',
                           'def command(cfg, trial):\n    raise RuntimeError("different capture")'))
            frozen[str(HERE.relative_to(ROOT))+'/suite.py'] = hashlib.sha256(old.read_bytes()).hexdigest()
            a.save(run/'source_hashes.json', frozen)
            with mock.patch.object(suite, 'hashes', return_value=current), self.assertRaises(RuntimeError):
                suite.verify_frozen(run)


class AutoResumeTests(unittest.TestCase):
    def group(self, runs, name, trials):
        path = runs/name
        path.mkdir()
        a.save(path/'progress.json', {'planned_trials': 5, 'trials': trials, 'state': 'STOPPED'})
        return path

    def test_newer_empty_group_does_not_hide_completed_first_trial(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            self.group(runs, '20261111_170356-failed', {'1': {'status': 'FAILED'}})
            expected = self.group(runs, '20261111_171817-valid', {'1': {'status': 'COMPLETE'}})
            self.group(runs, '20261111_174522-empty', {})
            self.assertEqual(suite.select_run(runs), expected)

    def test_newest_started_group_is_selected(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            self.group(runs, '20261111_171817-older', {'1': {'status': 'COMPLETE'}})
            expected = self.group(runs, '20261111_174522-started', {'1': {'status': 'FAILED'}})
            self.assertEqual(suite.select_run(runs), expected)

    def test_explicit_new_ignores_existing_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            self.group(runs, 'existing', {'1': {'status': 'COMPLETE'}})
            self.assertIsNone(suite.select_run(runs, start_new=True))

    def test_finished_group_is_not_replaced_with_old_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            self.group(runs, '20261111_170356-old', {'1': {'status': 'FAILED'}})
            expected = self.group(runs, '20261111_171817-finished',
                                  {str(i): {'status': 'COMPLETE'} for i in range(1, 6)})
            self.assertEqual(suite.select_run(runs), expected)

    def test_explicit_resume_overrides_automatic_choice(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            expected = self.group(runs, 'older', {})
            self.group(runs, 'newer', {'1': {'status': 'COMPLETE'}})
            self.assertEqual(suite.select_run(runs, resume=expected), expected)

    def test_external_resume_directory_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                suite.select_run(Path(tmp)/'runs', resume=Path(tmp)/'elsewhere')

    def test_no_history_means_new_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsNone(suite.select_run(Path(tmp)))

    def test_finished_group_main_exits_without_starting_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            here = Path(tmp)
            runs = here/'runs'
            runs.mkdir()
            group = self.group(runs, '20261111_171817-finished',
                               {str(i): {'status': 'COMPLETE'} for i in range(1, 6)})
            a.save(group/'config.json', a.load(HERE/'config.json'))
            with mock.patch.object(suite, 'HERE', here), mock.patch.object(sys, 'argv', ['suite.py']), \
                    mock.patch.object(sys.stdin, 'isatty', return_value=True), \
                    mock.patch.object(suite, 'read_session', return_value=(None, None)), \
                    mock.patch.object(suite, 'verify_frozen'), mock.patch.object(suite, 'run_trial') as capture:
                self.assertEqual(suite.main(), 0)
                capture.assert_not_called()
            self.assertEqual(a.load(group/'progress.json')['state'], 'FINISHED')


if __name__ == '__main__':
    unittest.main(verbosity=2)
