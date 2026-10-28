#!/usr/bin/python3
import argparse,subprocess,sys
from common import *
def main():
    p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--scene',required=True);p.add_argument('--scenes',default=str(ROOT/'config/scenes.yaml'));p.add_argument('--board',default=str(ROOT/'config/board.yaml'));p.add_argument('--frames',type=int,default=50);p.add_argument('--out',required=True);a=p.parse_args()
    b=board(a.board,require_confirm=False)
    if not b.get('measurements_confirmed'):raise ValueError('Board dimensions not confirmed')
    if not 1<=a.frames<=100:raise ValueError('Use 1..100 static frames')
    r=read(a.scenes)['scenes'][a.scene]['roi'];r={k:finite(v,k) for k,v in r.items()}
    for k in 'xyz':
        if r[k+'_min']>=r[k+'_max']:raise ValueError('Invalid ROI')
    out=output(a.out)
    if out.exists() and any(out.iterdir()):raise ValueError('Choose fresh empty diagnostic directory')
    out.mkdir(parents=True,exist_ok=True)
    exe=ROOT/'ws/install_integrated/fast_calib/lib/fast_calib/lidar_diagnostic'
    args=[str(exe),'--offline',str(Path(a.bag).resolve()),str(out)]+[str(r[k]) for k in ['x_min','x_max','y_min','y_max','z_min','z_max']]+[str(b[k]) for k in ['circle_radius','delta_width_circles','delta_height_circles']]+[str(a.frames)]
    with (out/'lidar_diagnostic.log').open('w') as f:proc=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,timeout=180)
    save(out/'diagnostic_manifest.json',{'bag':str(Path(a.bag).resolve()),'scene':a.scene,'roi':r,'board':b,'frames':a.frames,'returncode':proc.returncode,'extrinsics_solved':False})
    if (out/'edges.pcd').exists():
        subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/pcd_diagnostic.py'),str(out),'--board',a.board],check=True)
    if proc.returncode:
        save(out/'failure_advice.json',{'status':'FEATURE_EXTRACTION_FAILED','actions':['Check target lies inside 3D ROI; remove wall/background planes','Confirm all four holes are visible from LiDAR, with clear depth background','Keep board and rigid sensor assembly static, accumulate 5 seconds; compare 1/2/5/10 seconds','Move board closer if density is insufficient; avoid grazing incidence','Check plane RMS, 3cm neighborhoods and 5mm voxel leaf against sampling; do not tune thresholds merely to force a result'],'no_solve_performed':True})
    print(out);return proc.returncode
if __name__=='__main__':
    try:sys.exit(main() or 0)
    except (ValueError,KeyError,FileNotFoundError) as e:print('BLOCKED:',e,file=sys.stderr);sys.exit(2)
