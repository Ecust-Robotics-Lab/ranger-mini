#!/usr/bin/python3
import argparse, json, subprocess, sys, time, os
from pathlib import Path
import cv2, numpy as np, yaml
from common import *

def audit(a):
    data={t:{'stamps':[],'record_stamps':[],'frames':set(),'signatures':set()} for t in (LIDAR,IMAGE)}
    for t,m,ts in messages(a.bag):
        d=data[t];d['stamps'].append(stamp(m));d['record_stamps'].append(ts);d['frames'].add(m.header.frame_id)
        if t==LIDAR:
            sig=(tuple((f.name,f.offset,f.datatype,f.count) for f in m.fields),m.point_step,m.row_step==m.width*m.point_step,m.is_bigendian)
            d['signatures'].add(sig)
            xyz(m)  # Validate data layout and finite conversion, no alteration of source.
        else:
            d['signatures'].add((m.width,m.height,m.encoding,m.step))
            if len(m.data)!=m.height*m.step:raise ValueError('Malformed image buffer')
    result={'bag':str(Path(a.bag).resolve()),'topics':{}}
    issues=[]
    for t,d in data.items():
        s=d['stamps'];dt=np.diff(np.array(s,np.int64));n=len(s)
        result['topics'][t]={'count':n,'header_first_ns':s[0] if n else None,'header_last_ns':s[-1] if n else None,'header_duration_s':(s[-1]-s[0])/1e9 if n else 0,'record_first_ns':d['record_stamps'][0] if n else None,'record_last_ns':d['record_stamps'][-1] if n else None,'nonincreasing':int(sum(dt<=0)),'gaps_over_150ms':int(sum(dt>150000000)),'period_median_ms':float(np.median(dt)/1e6) if len(dt) else None,'frame_ids':sorted(d['frames']),'signatures':[list(x) for x in sorted(d['signatures'])]}
        if n<2 or (dt<=0).any() or (dt>150000000).any() or len(d['signatures'])!=1 or len(d['frames'])!=1:issues.append(t+': missing/gaps/format/frame changes')
    ls=data[LIDAR]['stamps'];ims=data[IMAGE]['stamps'];pairs=len(set(ls)&set(ims));result['exact_header_pairs']=pairs
    result['all_frames_exactly_paired']=pairs==len(ls)==len(ims) and pairs>0
    if not result['all_frames_exactly_paired']:issues.append('Header stamps not fully paired in Stage B common domain')
    if ls and ims:
        sl=np.sort(ls);dist=[]
        for s in ims:
            idx=np.searchsorted(sl,s);dist.append(min(abs(s-int(sl[j])) for j in (max(0,idx-1),min(len(sl)-1,idx))))
        result['nearest_stamp_error_max_ns']=max(dist)
    # Metadata counts are additional evidence, not proof of hardware FrameID continuity.
    meta=Path(a.bag)/'metadata.yaml'
    if meta.exists():
        md=read(meta)['rosbag2_bagfile_information'];mc={x['topic_metadata']['name']:x['message_count'] for x in md['topics_with_message_count']};result['metadata_counts']=mc
        if any(mc.get(t)!=len(data[t]['stamps']) for t in data):issues.append('Metadata count mismatch')
    result['issues']=issues;result['input_integrity_pass']=not issues
    result['physical_trigger_precision']='NOT_MEASURED';result['hardware_FrameID_continuity']='Needs driver CSV; Image has no FrameID counter'
    exe=ROOT/'ws/install_integrated/fast_calib/lib/fast_calib/input_probe'
    if exe.exists():
        p=subprocess.run([str(exe),str(Path(a.bag).resolve())],capture_output=True,text=True,timeout=60)
        result['fast_calib_reader']={'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr}
        if p.returncode:result['input_integrity_pass']=False
    else:result['fast_calib_reader']={'status':'NOT_BUILT'};result['input_integrity_pass']=False
    save(a.out,result);print(json.dumps(result,indent=2,ensure_ascii=False));return 0 if result['input_integrity_pass'] else 3

def import_intr(a):
    d=intrinsics(a.input)
    peer=Path(a.input).with_name('camera_info.yaml')
    if peer.exists() and peer.resolve()!=Path(a.input).resolve():
        other=intrinsics(peer)
        for k in ['image_width','image_height','K','D','distortion_model']:
            if d[k]!=other[k]:raise ValueError('camera_info mismatch: '+k)
        d['camera_info_sha256']=sha(peer)
    d['ready_for_solve']=d['status'] in ('PASS','PASSED','COMPLETE','VALIDATED')
    save(a.out,d);print('Imported real calibration; k3=',d['D'][4],'; status=',d['status'],'; ready=',d['ready_for_solve'])

def image_preview(a):
    if Path(a.input).is_dir():im,_,info=sample(a.input,1)
    else:
        im=cv2.imread(a.input);info={}
    if im is None:raise ValueError('Cannot read real image')
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    detector=cv2.aruco.DetectorParameters_create();detector.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    corners,ids,rejected=cv2.aruco.detectMarkers(gray,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),parameters=detector)
    flat=[] if ids is None else ids.ravel().tolist();vis=im.copy()
    if ids is not None:cv2.aruco.drawDetectedMarkers(vis,corners,ids)
    centers={};points={};clipped=[]
    for c,i in zip(corners,flat):
        c=c.reshape(4,2);centers[i]=c.mean(axis=0);points[i]=c.tolist()
        if (c[:,0]<8).any() or (c[:,0]>im.shape[1]-9).any() or (c[:,1]<8).any() or (c[:,1]>im.shape[0]-9).any():clipped.append(i)
        for j,(x,y) in enumerate(c):cv2.putText(vis,str(j),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
        cv2.circle(vis,tuple(centers[i].astype(int)),4,(255,0,255),-1)
    complete=sorted(flat)==[1,2,3,4]
    upright=False
    if complete:
        upright=bool(centers[1][0]<centers[2][0] and centers[3][0]<centers[4][0] and centers[1][1]<centers[3][1] and centers[2][1]<centers[4][1])
    orientations={str(i):bool(np.dot((c.reshape(4,2)[1]-c.reshape(4,2)[0]),np.array([1.,0.]))>0 and np.dot((c.reshape(4,2)[3]-c.reshape(4,2)[0]),np.array([0.,1.]))>0) for c,i in zip(corners,flat)}
    d={'marker_orientation_upright':orientations,'ids':flat,'complete_exact_ids':complete,'upright_layout_tl1_tr2_bl3_br4':upright,'corners':points,'centers':{str(k):v.tolist() for k,v in centers.items()},'near_image_boundary_ids':clipped,'global_saturated_fraction':float(np.mean(gray>=250)),'global_dark_fraction':float(np.mean(gray<=5)),'laplacian_variance':float(cv2.Laplacian(gray,cv2.CV_64F).var()),'image_width':im.shape[1],'image_height':im.shape[0],'info':info,'quality_note':'Exposure/blur metrics need visual review; global values include background','feature_match_verified':False}
    if a.intrinsics and a.board:
        intr=intrinsics(a.intrinsics);b=board(a.board);K=np.array(intr['K']);D=np.array(intr['D'])
        if (im.shape[1],im.shape[0])!=(intr['image_width'],intr['image_height']):raise ValueError('Resolution mismatch')
        obj,holes=board_geometry(b)
        if complete:
            objects=np.concatenate([obj[[1,2,4,3].index(i)] for i in flat]);pixels=np.concatenate(corners).reshape(-1,2)
            ok,rv,tv=cv2.solvePnP(objects,pixels,K,D)
            if ok:
                projected=cv2.projectPoints(objects,rv,tv,K,D)[0].reshape(-1,2)
                d['aruco_reprojection_rmse_px']=float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
                R=cv2.Rodrigues(rv)[0];d['camera_hole_centers']=((R@holes.T).T+tv.ravel()).tolist()
                for x,y in cv2.projectPoints(holes,rv,tv,K,D)[0].reshape(-1,2):cv2.circle(vis,(round(x),round(y)),6,(0,255,0),2)
    out=output(a.out);out.mkdir(exist_ok=True);cv2.imwrite(str(out/'aruco.png'),vis);save(out/'image_quality.json',d);print(json.dumps(d,indent=2));return 0 if complete and upright and all(orientations.values()) and not clipped else 3

def cloud_preview(a):
    im,p,info=sample(a.bag,a.frames);s=read(a.scenes)['scenes'][a.scene]['roi']
    for k in s:finite(s[k],k)
    lo=np.array([s[k+'_min'] for k in 'xyz']);hi=np.array([s[k+'_max'] for k in 'xyz'])
    if (lo>=hi).any():raise ValueError('Invalid ROI')
    p=p[((p>=lo)&(p<=hi)).all(axis=1)];out=output(a.out);out.mkdir(exist_ok=True)
    np.save(out/'roi_xyz.npy',p)
    with (out/'roi.pcd').open('w') as f:
        f.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA ascii\n'%(len(p),len(p)));np.savetxt(f,p,fmt='%.7g')
    d={'roi_points':len(p),'info':info,'roi':s,'circle_extraction':'Use lidar_diagnostic with validated board/config','feature_match_verified':False}
    if len(p)>=20:
        from scipy.spatial import cKDTree
        rng=np.random.default_rng(42);best=np.zeros(len(p),bool)
        fit=p[rng.choice(len(p),min(len(p),50000),replace=False)]
        for _ in range(150):
            v=fit[rng.choice(len(fit),3,replace=False)];n=np.cross(v[1]-v[0],v[2]-v[0]);norm=np.linalg.norm(n)
            if norm<1e-10:continue
            n/=norm;mask=np.abs(fit@n-v[0]@n)<.01
            if mask.sum()>best.sum():best=mask
        if best.sum()>=20:
            plane=fit[best];mu=plane.mean(0);_,sv,vh=np.linalg.svd(plane-mu,full_matrices=False);n=vh[-1];mask=np.abs((p-mu)@n)<.01;plane=p[mask]
            d.update(plane_points=len(plane),plane_inlier_fraction=float(mask.mean()),plane_rms_m=float(np.sqrt(np.mean(((plane-mu)@n)**2))),plane_distance_m=float(abs(mu@n)),median_neighbor_distance_m=float(np.median(cKDTree(plane).query(plane,k=2)[0][:,1])))
            np.save(out/'plane_xyz.npy',plane)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(13,5));ax=fig.add_subplot(121,projection='3d');q=p[::max(1,len(p)//30000)]
    if len(q):ax.scatter(*q.T,s=1,c=q[:,0],cmap='turbo');ax.set_xlabel('LiDAR x m');ax.set_ylabel('y m');ax.set_zlabel('z m')
    ax2=fig.add_subplot(122);ax2.imshow(cv2.cvtColor(im,cv2.COLOR_BGR2RGB));ax2.set_title('Source image (no projection)');fig.tight_layout();fig.savefig(out/'roi.png',dpi=160);plt.close(fig)
    save(out/'roi_quality.json',d);print(json.dumps(d,indent=2));return 0 if len(p)>=20 else 3

def make_config(a):
    b=board(a.board);intr=intrinsics(a.intrinsics)
    if intr['status'] not in ('PASS','PASSED','COMPLETE','VALIDATED') and not a.accept_partial_intrinsics:raise ValueError('Intrinsics status '+intr['status']+' requires explicit review; use --accept-partial-intrinsics only after review')
    s=read(a.scenes)['scenes'][a.scene]['roi'];s={k:finite(v,k) for k,v in s.items()}
    for k in 'xyz':
        if s[k+'_min']>=s[k+'_max']:raise ValueError('Invalid ROI')
    K=np.array(intr['K']);p={'configuration_validated':True,**s,**{k:float(b[k]) for k in DIMS},'fx':float(K[0,0]),'fy':float(K[1,1]),'cx':float(K[0,2]),'cy':float(K[1,2]),**dict(zip(['k1','k2','p1','p2','k3'],intr['D'])),'image_width':intr['image_width'],'image_height':intr['image_height'],'use_point_pick':False,'min_detected_markers':4,'max_frames':a.frames,'exit_after_solve':True,'preprocess_only':False,'bag_path':str(Path(a.bag).resolve()),'image_path':'','lidar_topic':LIDAR,'image_topic':IMAGE,'output_path':str(output(a.result_dir))}
    if not 1<=a.frames<=100:raise ValueError('Use 1..100 static frames')
    output(a.out).write_text(yaml.safe_dump(ros_parameter_document(p),sort_keys=False))
    save(str(a.out)+'.provenance.json',{'config_sha256':sha(a.out),'intrinsics':intr,'board_source':str(Path(a.board).resolve()),'board_sha256':sha(a.board),'scenes_sha256':sha(a.scenes),'scene':a.scene,'bag':p['bag_path'],'partial_intrinsics_explicitly_accepted':a.accept_partial_intrinsics})
    print(a.out)

def project(a):
    intr=intrinsics(a.intrinsics);T=np.array(read(a.transform)['T_cam_lidar']);check=matrix_check(T)
    im,p,info=paired_sample(a.bag,a.frames)
    if (im.shape[1],im.shape[0])!=(intr['image_width'],intr['image_height']):raise ValueError('Resolution mismatch')
    pc=(T[:3,:3]@p.T).T+T[:3,3];pc=pc[pc[:,2]>1e-4];K=np.array(intr['K']);D=np.array(intr['D'])
    uv=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2);mask=np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,0]<im.shape[1])&(uv[:,1]>=0)&(uv[:,1]<im.shape[0]);uv=uv[mask].astype(int);z=pc[mask,2]
    vis=im.copy()
    if len(z):
        colors=cv2.applyColorMap(np.uint8(np.clip(z/max(float(np.percentile(z,95)),1e-3),0,1)*255),cv2.COLORMAP_TURBO).reshape(-1,3)
        # Far to near, closest points overwrite far points; original distorted image.
        for idx in np.argsort(z)[::-1]:cv2.circle(vis,tuple(uv[idx]),1,tuple(map(int,colors[idx])),-1)
    out=output(a.out);out.mkdir(exist_ok=True);cv2.imwrite(str(out/'overlay.png'),vis)
    save(out/'projection.json',{**check,'visible_points':len(uv),'info':info,'projection_domain':'original distorted image; OpenCV plumb_bob with k3','reprojection_error_px':None,'note':'Overlay alone gives no correspondences or numerical reprojection accuracy'})

def parser():
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
    x=sub.add_parser('audit');x.add_argument('bag');x.add_argument('--out',required=True);x.set_defaults(fn=audit)
    x=sub.add_parser('import-intrinsics');x.add_argument('--input',default=str(ROOT.parent/'camera_intrinsics/results/camera_intrinsics.yaml'));x.add_argument('--out',default=str(ROOT/'config/intrinsics.imported.json'));x.set_defaults(fn=import_intr)
    x=sub.add_parser('image');x.add_argument('input');x.add_argument('--out',required=True);x.add_argument('--intrinsics');x.add_argument('--board');x.set_defaults(fn=image_preview)
    x=sub.add_parser('cloud');x.add_argument('bag');x.add_argument('--scene',required=True);x.add_argument('--scenes',default=str(ROOT/'config/scenes.yaml'));x.add_argument('--frames',type=int,default=50);x.add_argument('--out',required=True);x.set_defaults(fn=cloud_preview)
    x=sub.add_parser('make-config');x.add_argument('bag');x.add_argument('--scene',required=True);x.add_argument('--scenes',default=str(ROOT/'config/scenes.yaml'));x.add_argument('--board',default=str(ROOT/'config/board.yaml'));x.add_argument('--intrinsics',default=str(ROOT.parent/'camera_intrinsics/results/camera_intrinsics.yaml'));x.add_argument('--frames',type=int,default=50);x.add_argument('--result-dir',required=True);x.add_argument('--out',required=True);x.add_argument('--accept-partial-intrinsics',action='store_true');x.set_defaults(fn=make_config)
    x=sub.add_parser('project');x.add_argument('bag');x.add_argument('--intrinsics',required=True);x.add_argument('--transform',required=True);x.add_argument('--frames',type=int,default=1);x.add_argument('--out',required=True);x.set_defaults(fn=project)
    return p
if __name__=='__main__':
    try:
        a=parser().parse_args();sys.exit(a.fn(a) or 0)
    except (ValueError,KeyError,FileNotFoundError,RuntimeError) as e:print('BLOCKED:',e,file=sys.stderr);sys.exit(2)
