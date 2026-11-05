#!/usr/bin/env python3
"""Export exact native IMU-frame XYZ/I to NEW derived files; no bag changes."""
import argparse,importlib.util,json,sys
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('native_input',ROOT/'scripts/fast_livo2/backend/validate_motion.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',choices=['straight','turn'],default='straight')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    if not a.execute:print('DRY-RUN: export native clouds for diagnostic');return 0
    original=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
    native=original/a.capture/'livo_110000000/result/native_frames'
    samples_path=original/a.capture/('c2_new_map_recovery' if a.capture=='straight' else 'c1_new_map_optimized')/'samples.jsonl'
    samples=[r for r in map(json.loads,samples_path.read_text().splitlines()) if r['pose_valid']]
    by_stamp={r['stamp_ns']:r for r in samples};queries=[]
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'clouds').mkdir()
    for stamp,odom,cloud in b.load_pairs(native):
        if stamp not in by_stamp:continue
        row=by_stamp[stamp]
        assert cloud.header.frame_id=='fast_livo_imu' and not cloud.is_bigendian
        fields={f.name:f for f in cloud.fields}
        assert all(fields[f].datatype==7 and fields[f].count==1 for f in ['x','y','z','intensity'])
        assert len(cloud.data)==cloud.height*cloud.row_step and cloud.row_step>=cloud.width*cloud.point_step
        dtype=np.dtype({'names':['x','y','z','intensity'],'formats':['<f4']*4,
                        'offsets':[fields[f].offset for f in ['x','y','z','intensity']],'itemsize':cloud.point_step})
        view=np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=cloud.data,strides=(cloud.row_step,cloud.point_step))
        points=np.column_stack([view[n].ravel() for n in ['x','y','z','intensity']]).astype('<f4')
        assert np.isfinite(points).all()
        path=a.output/'clouds'/(str(row['index'])+'.pcd');n=len(points)
        header=f'# native IMU frame diagnostic export\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
        path.write_bytes(header.encode()+points.tobytes())
        assert np.array_equal(b.read_pcd(path),points)
        queries.append({'index':row['index'],'stamp_ns':stamp,'source_pcd':str(path.resolve()),
                        'source_pcd_sha256':b.sha(path),'prediction':row['quality']['prediction'],
                        'recorded_pose':row['pose_map_imu'],'recorded_hypotheses':row['quality']['native_diagnostics']['local_hypotheses'],
                        'target_keyframe_ids':row['quality']['native_diagnostics']['target_keyframe_ids']})
    assert len(queries)==len(samples)
    config_path=ROOT/'config/fast_livo2/localizer.yaml';config=yaml.safe_load(config_path.read_text())['ranger_fast_livo_localizer']['ros__parameters']
    archive=original/'new_local_map/snapshot'
    job={'kind':'controlled single-scan diagnostic; no Tag evidence used in registration','capture':a.capture,
         'archive':str(archive),'map_manifest_sha256':b.sha(archive/'manifest.json'),
         'compatibility':str(HERE.parent/'tag_two_capture_20261104/new_map_compatibility.json'),
         'config':config,'frozen_target_ids':queries[0]['target_keyframe_ids'],'queries':queries,
         'source_sha256':{str(x):b.sha(x) for x in [next(native.glob('*.db3')),samples_path,config_path]},
         'protocol_sha256':b.sha(HERE/'protocol.json')}
    (a.output/'job.json').write_text(json.dumps(job,indent=2)+'\n')
    print(json.dumps({'capture':a.capture,'queries':len(queries),'frozen_target_ids':job['frozen_target_ids']},indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
