#!/usr/bin/env python3
"""Offline fixed-Tag feasibility from saved C1 geometry; never registers a landmark."""
import argparse
import bisect
from collections import Counter
import importlib.util
import json
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('tag_extract', Path(__file__).with_name('extract_bag.py'))
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def lines(path):
    with Path(path).open() as stream:
        return [json.loads(row) for row in stream]


def rotation_distance(a, b):
    return float((Rotation.from_matrix(a[:3, :3]).inv()*Rotation.from_matrix(b[:3, :3])).magnitude())


def center(poses):
    result = np.eye(4)
    result[:3, 3] = np.median([p[:3, 3] for p in poses], axis=0)
    result[:3, :3] = Rotation.from_matrix(np.asarray([p[:3, :3] for p in poses])).mean().as_matrix()
    return result


def cluster(candidates, translation=.15, rotation=.15):
    """Complete-link modes; each image contributes at most one vote per mode."""
    modes = []
    for candidate in candidates:
        placed = False
        for mode in modes:
            if all(np.linalg.norm(candidate['T'][:3, 3]-other['T'][:3, 3]) <= translation
                   and rotation_distance(candidate['T'], other['T']) <= rotation for other in mode):
                if all(other['observation_id'] != candidate['observation_id'] for other in mode):
                    mode.append(candidate)
                placed = True
                break
        if not placed:
            modes.append([candidate])
    return sorted(modes, key=lambda mode: -len(mode))


class C1Sequence:
    def __init__(self, samples, native, map_uuid, version, gap=.15):
        self.samples = samples
        self.stamps = [row['stamp_ns'] for row in samples]
        require(all(b > a for a, b in zip(self.stamps, self.stamps[1:])), 'C1 duplicate/regressed stamps')
        self.native = {row['stamp_ns']: row for row in native}
        self.history = common.PoseHistory(native, gap)
        require(self.history.monotonic, 'native duplicate/regressed epoch')
        self.map_uuid, self.version, self.gap = map_uuid, version, gap

    def healthy(self, row):
        q = row.get('quality', {})
        require(row['state'] == 1 and row['pose_valid'] and row['last_confirmed_stamp_ns'] == row['stamp_ns']
                and row['reason'] == 'map_match_confirmed', 'C1 sample not geometrically confirmed')
        require(q.get('map_uuid') == self.map_uuid and q.get('graph_version') == self.version, 'C1 map/version mismatch')
        require(q.get('inputs_ready') and not q.get('map_loading') and q.get('converged')
                and q.get('geometrically_accepted'), 'C1 geometry or input health invalid')
        require(q['rmse_m'] <= .2 and q['source_overlap'] >= .45 and q['correspondences'] >= 30
                and q['normalized_plane_hessian_eigen_ratio'] >= 1e-4, 'C1 existing quality gate failed')
        require(row['stamp_ns'] in self.native, 'C1 stamp not in bound native trajectory')
        return common.matrix(q['T_target_source'])

    def camera(self, tc, E):
        association = self.history.lookup(tc)
        require(association['available'], 'native pose cannot bracket image: '+association.get('reason', ''))
        index = bisect.bisect_left(self.stamps, tc)
        require(2 <= index < len(self.samples), 'insufficient C1 segment or no following sample')
        selected = self.samples[index]
        require(0 <= selected['stamp_ns']-tc <= 300_000_000, 'C1 attachment age exceeded')
        segment = self.samples[index-2:index+1]
        corrections = []
        for row in segment:
            X = self.healthy(row)
            U = common.matrix(self.native[row['stamp_ns']]['pose'])
            corrections.append(X@np.linalg.inv(U))
        for a, b, ca, cb in zip(segment, segment[1:], corrections, corrections[1:]):
            require((b['stamp_ns']-a['stamp_ns'])*1e-9 <= self.gap, 'C1 trusted segment time gap')
            require(np.linalg.norm(ca[:3, 3]-cb[:3, 3]) <= .25 and rotation_distance(ca, cb) <= .15,
                    'C1 trusted segment correction discontinuity')
        X = self.healthy(selected)
        Us = common.matrix(self.native[selected['stamp_ns']]['pose'])
        Uc = common.matrix(association['pose'])
        camera = X@np.linalg.inv(Us)@Uc@E
        return camera, {'scan_stamp_ns': selected['stamp_ns'], 'scan_index': selected['index'],
                        'image_stamp_ns': tc, 'attachment_delta_ns': selected['stamp_ns']-tc,
                        'native_interpolation': association,
                        'segment_scan_stamps_ns': [row['stamp_ns'] for row in segment],
                        'C1_map_pose': common.pose_json(X), 'C1_quality': selected['quality'],
                        'saved_C1_session_and_request_ids_available': False}


def object_points(size):
    return np.asarray([[-size/2, size/2, 0], [size/2, size/2, 0],
                       [size/2, -size/2, 0], [-size/2, -size/2, 0]], dtype=np.float64)


def reproject(transform, camera_inverse, observation, camera_model):
    T = camera_inverse@transform
    points = object_points(observation['size_m'])
    camera_points = points@T[:3, :3].T+T[:3, 3]
    require(np.isfinite(camera_points).all() and np.min(camera_points[:, 2]) > 1e-6, 'nonpositive Tag corner depth')
    projected, _ = cv2.projectPoints(points, Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3],
                                    np.asarray(camera_model['K']).reshape(3, 3), np.asarray(camera_model['D']))
    return projected[:, 0, :]-np.asarray(observation['corners'])


def frame_errors(transform, cameras, observations, model):
    result = []
    for camera, row in zip(cameras, observations):
        try:
            errors = reproject(transform, np.linalg.inv(camera), row['observation'], model)
            lengths = np.linalg.norm(errors, axis=1)
            result.append({'image_index': row['image_index'], 'rmse_px': float(np.sqrt(np.mean(lengths**2))),
                           'maximum_px': float(np.max(lengths)), 'positive_depth': True})
        except ValueError as error:
            result.append({'image_index': row['image_index'], 'rmse_px': None, 'maximum_px': None,
                           'positive_depth': False, 'reason': str(error)})
    return result


def fit_fixed_pose(seed, cameras, observations, model, train_count, maximum_evaluations=150):
    """Only one Tag pose is varied. C1 trajectory, calibration and time stay fixed."""
    inverse_cameras = [np.linalg.inv(camera) for camera in cameras]

    def decode(x):
        result = np.eye(4)
        result[:3, 3] = x[:3]
        result[:3, :3] = Rotation.from_rotvec(x[3:]).as_matrix()
        return result

    def residual(x):
        transform = decode(x)
        errors = []
        for i in range(train_count):
            try:
                errors.extend(reproject(transform, inverse_cameras[i], observations[i]['observation'], model).ravel())
            except ValueError:
                errors.extend([1e4]*8)
        return np.asarray(errors)

    initial = np.r_[seed[:3, 3], Rotation.from_matrix(seed[:3, :3]).as_rotvec()]
    fitted = least_squares(residual, initial, loss='huber', f_scale=1., max_nfev=maximum_evaluations)
    transform = decode(fitted.x)
    errors = frame_errors(transform, cameras, observations, model)
    return {'pose': common.pose_json(transform), 'optimizer_success': bool(fitted.success), 'evaluations': fitted.nfev,
            'train_image_indices': [row['image_index'] for row in observations[:train_count]],
            'holdout_image_indices': [row['image_index'] for row in observations[train_count:]],
            'train_rmse_px': common.quantiles([row['rmse_px'] for row in errors[:train_count]]),
            'holdout_rmse_px': common.quantiles([row['rmse_px'] for row in errors[train_count:]]), 'frame_errors': errors,
            'loss': 'Huber, 1 px coordinate scale; diagnostic, not field calibrated',
            'trajectory_optimized': False, 'time_optimized': False}


def independent_views(cameras, translation=.15, rotation=.08):
    selected = []
    for i, camera in enumerate(cameras):
        if all(np.linalg.norm(camera[:3, 3]-cameras[j][:3, 3]) >= translation
               or rotation_distance(camera, cameras[j]) >= rotation for j in selected):
            selected.append(i)
    return selected


def mode_details(mode, cameras, observations, model):
    poses = [row['T'] for row in mode]
    transform = center(poses)
    positions = [float(np.linalg.norm(p[:3, 3]-transform[:3, 3])) for p in poses]
    rotations = [rotation_distance(p, transform) for p in poses]
    return {'pose': common.pose_json(transform), 'observation_count': len(mode),
            'members': [{'observation_id': row['observation_id'], 'image_index': row['image_index'],
                         'hypothesis_id': row['hypothesis_id']} for row in mode],
            'translation_from_center_m': common.quantiles(positions),
            'rotation_from_center_deg': common.quantiles(np.rad2deg(rotations)),
            'fixed_center_reprojection': frame_errors(transform, cameras, observations, model)}


def evaluate_offset(sequence, observations, config, offset_ns, train_count):
    cameras, support, candidates = [], [], []
    for index, row in enumerate(observations):
        observation = row['observation']
        tc = observation['image_stamp_ns']+offset_ns
        camera, record = sequence.camera(tc, common.matrix(config['T_imu_camera']))
        cameras.append(camera)
        support.append(record)
        for hypothesis in observation['hypotheses']:
            candidates.append({'T': camera@common.matrix(hypothesis['T_camera_tag']), 'image_index': row['image_index'],
                               'observation_id': observation['observation_id'], 'hypothesis_id': hypothesis['hypothesis_id'], 'index': index})
    modes = cluster(candidates)
    training_modes = cluster([candidate for candidate in candidates if candidate['index'] < train_count])
    require(len(training_modes) <= 8, 'too many training pose modes; assessment budget exhausted')
    fitted = [fit_fixed_pose(center([candidate['T'] for candidate in mode]), cameras, observations,
                             config['camera'], train_count) for mode in training_modes]
    maximum_baseline = max(float(np.linalg.norm(a[:3, 3]-b[:3, 3])) for a in cameras for b in cameras)
    maximum_rotation = max(rotation_distance(a, b) for a in cameras for b in cameras)
    origin = center([candidate['T'] for candidate in modes[0]])[:3, 3]
    rays = [(camera[:3, 3]-origin)/np.linalg.norm(camera[:3, 3]-origin) for camera in cameras]
    parallax = max(float(np.arccos(np.clip(np.dot(a, b), -1, 1))) for a in rays for b in rays)
    for fit in fitted:
        fit['all_frames_pass_current_pixel_limits'] = all(
            row['positive_depth'] and row['rmse_px'] <= config.get('pixel_rmse', 3.)
            and row['maximum_px'] <= config.get('pixel_max', 6.) for row in fit['frame_errors'])
    mode_comparisons = []
    for i in range(len(modes)):
        for j in range(i+1, len(modes)):
            a, b = center([c['T'] for c in modes[i]]), center([c['T'] for c in modes[j]])
            mode_comparisons.append({'a': i, 'b': j, 'translation_m': float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                                     'rotation_deg': float(np.rad2deg(rotation_distance(a, b)))})
    return {'assumed_offset_ns': offset_ns, 'offset_is_calibrated': False,
            'map_cameras': [common.pose_json(camera) for camera in cameras], 'C1_support': support,
            'candidate_observations': [{'image_index': c['image_index'], 'observation_id': c['observation_id'],
                                       'hypothesis_id': c['hypothesis_id'], 'T_map_tag': common.pose_json(c['T'])} for c in candidates],
            'modes': [mode_details(mode, cameras, observations, config['camera']) for mode in modes],
            'mode_comparisons': mode_comparisons, 'fixed_trajectory_train_holdout_fits': fitted,
            'maximum_camera_translation_baseline_m': maximum_baseline,
            'maximum_camera_rotation_deg': float(np.rad2deg(maximum_rotation)),
            'maximum_view_ray_change_deg': float(np.rad2deg(parallax)),
            'independent_support_existing_motion_rule': independent_views(cameras),
            'independent_support_translation_only': independent_views(cameras, rotation=10.),
            'assessed_observations': len(observations), 'formal_registration_allowed': False}


def verify_map(directory, expected_sha):
    manifest_path = directory/'manifest.json'
    digest = common.sha_file(manifest_path)
    require(digest == expected_sha, 'source map SHA differs from saved C1 source')
    require(json.loads((directory/'COMMITTED').read_text())['manifest_sha256'] == digest, 'source map COMMITTED mismatch')
    manifest = json.loads(manifest_path.read_text())
    require(manifest['schema_version'] == 1 and manifest['kind'] == 'stage_b_graph_archive', 'source map schema')
    for relative, digest in manifest['files'].items():
        path = directory/relative
        require(not Path(relative).is_absolute() and '..' not in Path(relative).parts and not path.is_symlink(), 'unsafe map path')
        require(common.sha_file(path) == digest, 'source map asset changed: '+relative)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--extraction', type=Path, default=ROOT/'outputs/fast-livo-apriltag-fullbag-20261029')
    parser.add_argument('--config', type=Path, default=ROOT/'config/fast_livo2/apriltag_camera.json')
    parser.add_argument('--fixedness-declaration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'scope': 'assessment only; no registration', 'output': str(args.output)}))
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    begin = time.monotonic()
    extraction_summary = json.loads((args.extraction/'summary.json').read_text())
    require(extraction_summary['passed'], 'source extraction not verified')
    extraction_manifest = json.loads((args.extraction/'manifest.json').read_text())
    require(json.loads((args.extraction/'COMMITTED').read_text())['manifest_sha256'] == common.sha_file(args.extraction/'manifest.json'), 'extraction COMMITTED mismatch')
    for name in ['observations.jsonl', 'replay_odometry.jsonl', 'frames.jsonl', 'summary.json']:
        require(common.sha_file(args.extraction/name) == extraction_manifest['files'][name], 'extraction asset SHA mismatch: '+name)
    config = json.loads(args.config.read_text())
    require(common.sha_file(args.config) == extraction_summary['config_sha256'], 'assessment calibration differs from extraction')
    require(config['camera']['pixel_domain'] == 'raw' and config['camera']['distortion_model'] == 'plumb_bob', 'assessment camera domain unsupported')
    observations = [row for row in lines(args.extraction/'observations.jsonl') if row['observation']['family'] == 'tag36h11' and row['observation']['id'] == 1]
    require(len(observations) >= 4, 'insufficient Tag observations for train/holdout assessment')
    require(all(row['metric_quality_eligible'] for row in observations), 'some observations did not pass original pixel quality')
    declaration = json.loads(args.fixedness_declaration.read_text())
    require(declaration['family'] == 'tag36h11' and declaration['id'] == 1, 'fixedness identity mismatch')
    native = lines(args.extraction/'replay_odometry.jsonl')
    frames = lines(args.extraction/'frames.jsonl')
    period_ns = int(np.median(np.diff([row['image_stamp_ns'] for row in frames])))
    offsets = [i*(period_ns//4) for i in range(-8, 9)]
    train_count = (2*len(observations))//3
    provenance = {'extraction_manifest_sha256': common.sha_file(args.extraction/'manifest.json'),
                  'config_sha256': common.sha_file(args.config), 'fixedness_declaration_sha256': common.sha_file(args.fixedness_declaration)}
    result = {'schema_version': 1, 'kind': 'apriltag_landmark_assessment_not_map_extension',
              'fixedness_declaration': declaration, 'physical_time_verified': False, 'global_pose_submitted': False,
              'real_VALID_landmark_created': False, 'source_provenance': provenance, 'maps': {},
              'time_sensitivity': {'offsets_ns': offsets, 'derived_image_period_ns': period_ns,
                                   'definition': '+/-2 observed sampling periods in quarter-period steps; diagnostic only'},
              'train_count': train_count, 'holdout_count': len(observations)-train_count}
    with (args.output/'time_sensitivity.jsonl').open('w') as sensitivity:
        for method in ['pcl', 'gicp']:
            source = ROOT/f'outputs/fast-livo-c1-final-20261028-{method}'
            saved = json.loads((source/'result.json').read_text())
            require(saved['passed'] and saved['source_database_sha256'] == extraction_summary['replay_identity']['sha256'], 'saved C1/native source identity mismatch')
            require(saved['source_database_sha256'] == common.sha_file(Path(saved['source_native'])/'native_frames_0.db3'), 'native database changed')
            map_dir = ROOT/f'outputs/fast-livo-backend-validation-20261028-{method}/snapshot'
            manifest = verify_map(map_dir, saved['source_manifest_sha256'])
            samples = lines(source/'samples.jsonl')
            sequence = C1Sequence(samples, native, manifest['map_uuid'], manifest['graph_version'])
            baseline = evaluate_offset(sequence, observations, config, 0, train_count)
            (args.output/f'{method}_zero_offset.json').write_text(json.dumps(baseline, indent=2)+'\n')
            sweep = []
            for offset in offsets:
                try:
                    evaluation = baseline if offset == 0 else evaluate_offset(sequence, observations, config, offset, train_count)
                    row = {'method': method, 'assumed_offset_ns': offset, 'available': True,
                           'mode_count': len(evaluation['modes']),
                           'fits': [{'seed_mode': i, 'optimizer_success': fit['optimizer_success'],
                                     'train_rmse_px': fit['train_rmse_px'], 'holdout_rmse_px': fit['holdout_rmse_px'],
                                     'all_frames_pass_current_pixel_limits': fit['all_frames_pass_current_pixel_limits']}
                                    for i, fit in enumerate(evaluation['fixed_trajectory_train_holdout_fits'])],
                           'offset_is_calibrated': False}
                except ValueError as error:
                    row = {'method': method, 'assumed_offset_ns': offset, 'available': False, 'reason': str(error), 'offset_is_calibrated': False}
                sensitivity.write(json.dumps(row)+'\n')
                sweep.append(row)
            conditions = {'fixedness_user_confirmed': bool(declaration.get('fixed_during_capture')),
                          'C1_all_observations_have_trusted_segment': baseline['assessed_observations'] == len(observations),
                          'physical_time_verified': False,
                          'unique_pose_mode_at_zero_offset': len(baseline['modes']) == 1,
                          'single_fixed_pose_explains_all_zero_offset_observations': any(fit['all_frames_pass_current_pixel_limits'] for fit in baseline['fixed_trajectory_train_holdout_fits'])}
            result['maps'][method] = {'map_uuid': manifest['map_uuid'], 'graph_version': manifest['graph_version'],
                                      'source_manifest_sha256': saved['source_manifest_sha256'], 'map_gravity_aligned': False,
                                      'source_C1_result_sha256': common.sha_file(source/'result.json'),
                                      'source_C1_samples_sha256': common.sha_file(source/'samples.jsonl'),
                                      'source_C1_saved_session_request_ids_available': False,
                                      'conditions': conditions, 'assessment': 'NOT_READY_FOR_VALID_REGISTRATION',
                                      'geometric_state': 'AMBIGUOUS' if len(baseline['modes']) > 1 else 'PROVISIONAL',
                                      'zero_offset_file': method+'_zero_offset.json',
                                      'zero_offset_mode_count': len(baseline['modes']),
                                      'mode_comparisons': baseline['mode_comparisons'],
                                      'maximum_camera_translation_baseline_m': baseline['maximum_camera_translation_baseline_m'],
                                      'maximum_camera_rotation_deg': baseline['maximum_camera_rotation_deg'],
                                      'maximum_view_ray_change_deg': baseline['maximum_view_ray_change_deg'],
                                      'independent_support_existing_motion_rule': len(baseline['independent_support_existing_motion_rule']),
                                      'independent_support_translation_only': len(baseline['independent_support_translation_only']),
                                      'zero_offset_fits': [{key: value for key, value in fit.items() if key != 'frame_errors'} for fit in baseline['fixed_trajectory_train_holdout_fits']],
                                      'sweep_evaluated_offsets': len(sweep)}
    result['evaluation_completed'] = True
    result['registration_eligible'] = False
    result['wall_seconds'] = time.monotonic()-begin
    result['limitations'] = ['Single capture and a 1.7-second correlated window, not independent visits',
                             'C1 and B use the same capture; no independent map accuracy truth',
                             'No map-pose interpolation: only native local trajectory interpolation',
                             'Fixed-trajectory Tag fitting is diagnostic; no frontend, C1, map or calibration update',
                             'Offset sensitivity does not establish a physical time mapping',
                             'Old C1 saved samples omit explicit session/request IDs; these are not invented',
                             'Old maps are not gravity aligned; map z is not external vertical height']
    (args.output/'assessment.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
