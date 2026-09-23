#!/usr/bin/env python3
"""Check complete Lightning output; geometric quality still needs measured references."""
import argparse,json,struct
from pathlib import Path
import numpy as np
import yaml

def lzf(data, expected):
    out=bytearray();i=0
    while i<len(data):
        ctrl=data[i];i+=1
        if ctrl<32:
            n=ctrl+1;out.extend(data[i:i+n]);i+=n
        else:
            n=ctrl>>5;off=(ctrl&31)<<8
            if n==7:n+=data[i];i+=1
            off+=data[i];i+=1;ref=len(out)-off-1
            if ref<0:raise ValueError('Invalid LZF reference')
            for k in range(n+2):out.append(out[ref+k])
        if len(out)>expected:raise ValueError('Invalid LZF output length')
    if len(out)!=expected:raise ValueError('Truncated LZF')
    return out

def pcd_xyz(path):
    with path.open('rb') as f:
        h={}
        while True:
            line=f.readline()
            if not line:raise ValueError('No PCD DATA header')
            words=line.decode('ascii').strip().split()
            if not words or words[0].startswith('#'):continue
            h[words[0]]=words[1:]
            if words[0]=='DATA':break
        payload=f.read()
    fields=h['FIELDS'];sizes=list(map(int,h['SIZE']));counts=list(map(int,h.get('COUNT',['1']*len(fields))))
    n=int(h['POINTS'][0]);kind=h['DATA'][0]
    if n<1:raise ValueError('Empty PCD')
    if kind=='ascii':
        a=np.loadtxt(payload.decode().splitlines(),ndmin=2)
        offsets=np.cumsum([0]+counts[:-1]);xyz=a[:,[offsets[fields.index(k)] for k in ['x','y','z']]]
    elif kind=='binary_compressed':
        packed,raw_size=struct.unpack('<II',payload[:8])
        if packed!=len(payload)-8:raise ValueError('PCD compressed size mismatch')
        raw=lzf(payload[8:],raw_size);offset=0;cols={}
        for key,size,count,typ in zip(fields,sizes,counts,h['TYPE']):
            width=n*size*count
            if key in ['x','y','z']:
                if typ!='F' or count!=1 or size not in [4,8]:raise ValueError('Unsupported XYZ field')
                cols[key]=np.frombuffer(raw,dtype='<f'+str(size),count=n,offset=offset)
            offset+=width
        if offset!=raw_size:raise ValueError('PCD field sizes mismatch')
        xyz=np.column_stack([cols[k] for k in ['x','y','z']])
    elif kind=='binary':
        dtype=np.dtype([(key,('<'+{'F':'f','I':'i','U':'u'}[typ]+str(size)),(count,)) for key,size,count,typ in zip(fields,sizes,counts,h['TYPE'])])
        arr=np.frombuffer(payload,dtype=dtype,count=n);xyz=np.column_stack([arr[k][:,0] for k in ['x','y','z']])
    else:raise ValueError('Unsupported PCD encoding')
    if len(xyz)!=n or not np.isfinite(xyz).all():raise ValueError('PCD count/nonfinite coordinates')
    return xyz

def inspect(exp):
    folder=exp/'data/new_map';index=folder/'index.txt'
    if not index.is_file():raise ValueError('Missing localization map index.txt')
    lines=index.read_text().splitlines();rows=[];has_start=False
    for line in lines[1:]:
        p=line.split()
        if not p or line.startswith('#'):continue
        if p[0]=='start':
            if len(p)!=8 or not np.isfinite(list(map(float,p[1:]))).all():raise ValueError('Invalid start pose')
            has_start=True;continue
        if len(p)!=4 or not p[0].isdigit():raise ValueError('Unknown index row')
        path=folder/(p[0]+'.pcd');xyz=pcd_xyz(path);rows.append({'file':path.name,'points':len(xyz)})
    if not rows or not has_start:raise ValueError('Incomplete localization map')
    trajectory=np.loadtxt(folder/'keyframes.txt',ndmin=2)
    if trajectory.shape[1]!=16 or not np.isfinite(trajectory).all():raise ValueError('Incomplete 6DoF keyframe trajectory')
    xyz=pcd_xyz(folder/'global.pcd')
    result={'structural_check':'PASS','chunks':rows,'global_points':len(xyz),'keyframes':len(trajectory),
            'bounds_min_m':xyz.min(axis=0).tolist(),'bounds_max_m':xyz.max(axis=0).tolist(),
            'centroid_m':xyz.mean(axis=0).tolist(),'geometric_acceptance':'PENDING_MANUAL_AND_REPRODUCTION'}
    cfg=yaml.safe_load((exp/'config.yaml').read_text())
    if cfg['system']['with_g2p5']:
        meta=yaml.safe_load((folder/'map.yaml').read_text())
        if abs(meta['resolution']-cfg['g2p5']['grid_map_resolution'])>1e-6:raise ValueError('2D resolution mismatch')
        import cv2
        im=cv2.imread(str(folder/meta['image']),cv2.IMREAD_UNCHANGED)
        if im is None or list(im.shape)!=[meta['height'],meta['width']]:raise ValueError('2D image dimensions mismatch')
        if not np.isfinite(meta['origin']).all():raise ValueError('Invalid 2D origin')
        vals,counts=np.unique(im,return_counts=True)
        result['grid']={'resolution':meta['resolution'],'shape':list(im.shape),'values':dict(zip(map(str,vals),map(int,counts)))}
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('experiment',type=Path);ap.add_argument('--compare',type=Path);a=ap.parse_args()
    try:
        result=inspect(a.experiment)
        if a.compare:
            other=inspect(a.compare)
            ma=json.loads((a.experiment/'manifest.json').read_text());mb=json.loads((a.compare/'manifest.json').read_text())
            if ma['bag_inventory']!=mb['bag_inventory']:raise ValueError('Reproduction requires identical bag hashes')
            for key in ['current_source_commit','source_files_sha256','executable_sha256','built_libraries_sha256']:
                if ma.get(key)!=mb.get(key):raise ValueError('Reproduction source/build changed: '+key)
            if (a.experiment/'config.yaml').read_bytes()!=(a.compare/'config.yaml').read_bytes():raise ValueError('Reproduction requires identical config')
            from scipy.spatial import cKDTree
            x=pcd_xyz(a.experiment/'data/new_map/global.pcd');y=pcd_xyz(a.compare/'data/new_map/global.pcd')
            d=np.r_[cKDTree(x).query(y)[0],cKDTree(y).query(x)[0]]
            result['reproduction']={'other':str(a.compare),'points':[len(x),len(y)],'symmetric_nn_median_m':float(np.median(d)),
                'symmetric_nn_p95_m':float(np.percentile(d,95)),'symmetric_nn_max_m':float(d.max()),'acceptance':'REVIEW_REQUIRED'}
        code=0
    except Exception as e:result={'structural_check':'FAIL','error':str(e)};code=2
    (a.experiment/('comparison.json' if a.compare else 'map_check.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return code
if __name__=='__main__':raise SystemExit(main())
