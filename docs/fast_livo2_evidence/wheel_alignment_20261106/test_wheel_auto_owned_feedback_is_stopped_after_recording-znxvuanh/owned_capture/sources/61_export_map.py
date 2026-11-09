#!/usr/bin/env python3
"""Validate a B archive, then export a new, source-bound navigation map candidate."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(Path(__file__).parent)]
from check_slam_map import pcd_xyz
from grid import floor_plane,make_grid


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive',type=Path,required=True);p.add_argument('--identity-file',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true')
    p.add_argument('--resolution',type=float,default=.05);p.add_argument('--maximum-range',type=float,default=12.)
    p.add_argument('--obstacle-min',type=float,default=.1);p.add_argument('--obstacle-max',type=float,default=1.2)
    args=p.parse_args();archive=args.archive.resolve();output=args.output.absolute()
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','archive':str(archive),'output':str(output),'hardware_started':False}));return 0
    if output.exists(): raise ValueError('output must be a new directory')
    prefix=ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer'
    checked=subprocess.run([str(prefix/'lib/ranger_fast_livo_localizer/archive_tool'),'--archive',str(archive),
                            '--identity-file',str(args.identity_file.resolve())],capture_output=True,text=True)
    if checked.returncode: raise RuntimeError('archive validation failed: '+checked.stderr)
    manifest=json.loads((archive/'manifest.json').read_text());source_sha=sha(archive/'manifest.json')
    positions=np.array([f['X_map_imu']['xyz'] for f in manifest['keyframes']])
    plane,floor_quality=floor_plane(pcd_xyz(archive/'map.pcd'),positions)
    frames=[(pcd_xyz(archive/f['directory']/'full.pcd'),f['X_map_imu']) for f in manifest['keyframes']]
    values,origin,counts=make_grid(frames,plane,args.resolution,args.maximum_range,args.obstacle_min,args.obstacle_max)
    if sha(archive/'manifest.json')!=source_sha: raise RuntimeError('source map changed during export')
    output.mkdir(parents=True)
    try:
        pixels=np.full(values.shape,205,np.uint8);pixels[values==0]=254;pixels[values==100]=0
        Image.fromarray(np.flipud(pixels)).save(output/'map.pgm')
        map_yaml={'image':'map.pgm','mode':'trinary','resolution':args.resolution,
                  'origin':[float(origin[0]),float(origin[1]),0.], 'negate':0,'occupied_thresh':.65,'free_thresh':.196}
        (output/'map.yaml').write_text(yaml.safe_dump(map_yaml,sort_keys=False))
        result={'schema_version':1,'kind':'fast_livo_navigation_map_candidate','map_uuid':manifest['map_uuid'],
                'source_archive':str(archive),'source_manifest_sha256':source_sha,'graph_version':manifest['graph_version'],
                'frame':'map','projection':'optimized keyframes transformed once; planar navigation image only',
                'floor_plane_abc':plane.tolist(),'floor_quality':floor_quality,'resolution':args.resolution,
                'origin':map_yaml['origin'],'width':values.shape[1],'height':values.shape[0],'cells':counts,
                'parameters':{'maximum_range_m':args.maximum_range,'obstacle_height_m':[args.obstacle_min,args.obstacle_max],
                              'ground_vertical_band_m':[-.08,.08],'ray_bins':720,'minimum_obstacle_hits':1,'fusion':'per-keyframe hit/miss evidence; occupied/free thresholds in cells.evidence_fusion'},
                'map_image_sha256':sha(output/'map.pgm'),'map_yaml_sha256':sha(output/'map.yaml'),
                'generator_sha256':{str(f.relative_to(ROOT)):sha(f) for f in [Path(__file__),Path(__file__).with_name('grid.py')]},
                'field_navigation_acceptance':'NOT_EVALUATED','source_physical_sync_verified':False,
                'limitations':['Floor estimated from this map; actual floor and body-height band need field review.',
                               'Observed free rays do not certify full swept-body clearance.',
                               'Unknown space remains unknown; no start-area obstacle erasing.']}
        (output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
        (output/'archive_validation.json').write_text(checked.stdout)
        (output/'COMMITTED').write_text(json.dumps({'manifest_sha256':sha(output/'manifest.json')})+'\n')
        print(json.dumps(result,indent=2));return 0
    except Exception as exc:
        (output/'FAILED.json').write_text(json.dumps({'reason':str(exc)})+'\n');raise


if __name__=='__main__':raise SystemExit(main())
