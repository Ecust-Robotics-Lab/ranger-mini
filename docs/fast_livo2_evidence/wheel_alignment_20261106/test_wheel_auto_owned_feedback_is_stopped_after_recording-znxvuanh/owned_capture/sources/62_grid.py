"""Flat-floor navigation projection; the source 3D map is never modified."""
import numpy as np
from scipy.spatial import cKDTree


def floor_plane(points, positions, seed=47):
    # Use points below nearby sensor poses, excluding ceiling and distant floors.
    _, nearest = cKDTree(positions[:, :2]).query(points[:, :2])
    below = points[:, 2]-positions[nearest, 2]
    distance = np.linalg.norm(points[:, :2]-positions[nearest, :2], axis=1)
    candidates = points[(below < -.3) & (below > -1.2) & (distance < 3.)]
    if len(candidates) < 100: raise ValueError('insufficient nearby floor observations')
    rng = np.random.default_rng(seed)
    sample = candidates[rng.choice(len(candidates), min(12000,len(candidates)), replace=False)]
    best = None
    for _ in range(160):
        triple = sample[rng.choice(len(sample),3,replace=False)]
        a = np.column_stack((triple[:,:2],np.ones(3)))
        if abs(np.linalg.det(a)) < .01: continue
        plane = np.linalg.solve(a,triple[:,2])
        if np.linalg.norm(plane[:2]) > np.tan(np.deg2rad(7.)): continue
        residual = np.abs(sample[:,2]-(sample[:,:2]@plane[:2]+plane[2]))
        keep = residual < .05
        score = int(keep.sum())
        if best is None or score > best[0]: best = score,plane,keep
    if best is None or best[0] < .15*len(sample): raise ValueError('no supported near-horizontal floor')
    selected = sample[best[2]]
    plane = np.linalg.lstsq(np.column_stack((selected[:,:2],np.ones(len(selected)))),selected[:,2],rcond=None)[0]
    residual = np.abs(selected[:,2]-(selected[:,:2]@plane[:2]+plane[2]))
    return plane, {'candidate_points':len(candidates),'sample_points':len(sample),'inliers':len(selected),
                   'inlier_fraction':len(selected)/len(sample),'inlier_vertical_residual_p95_m':float(np.percentile(residual,95)),
                   'model':'map_z = a*map_x + b*map_y + c; projection only, source map unchanged',
                   'field_floor_verified':False}


def transform(points, pose):
    from scipy.spatial.transform import Rotation
    return points@Rotation.from_quat(pose['xyzw']).as_matrix().T+np.asarray(pose['xyz'])


def raytrace_free(occupied, origin_cell, endpoints):
    """DDA rays stop at this observation's occupied cells; unseen cells stay unknown."""
    height,width=occupied.shape
    start=np.asarray(origin_cell,dtype=int)
    if not (0<=start[0]<width and 0<=start[1]<height): raise ValueError('sensor origin outside grid')
    endpoints=np.asarray(endpoints,dtype=int).reshape(-1,2)
    free=np.zeros_like(occupied,dtype=bool)
    if not len(endpoints): return free
    delta=endpoints-start
    count=np.max(np.abs(delta),axis=1)
    steps=np.arange(int(count.max())+1)
    fraction=np.minimum(steps[None,:]/np.maximum(count[:,None],1),1.)
    cells=np.floor(start[None,None,:]+fraction[:,:,None]*delta[:,None,:]+.5).astype(int)
    xs,ys=cells[:,:,0],cells[:,:,1]
    valid=(steps[None,:]<=count[:,None])&(xs>=0)&(xs<width)&(ys>=0)&(ys<height)
    blocked=np.ones_like(valid)
    blocked[valid]=occupied[ys[valid],xs[valid]]
    visible=valid & ~np.maximum.accumulate(blocked,axis=1)
    free[ys[visible],xs[visible]]=True
    return free


def make_grid(frames, plane, resolution=.05, maximum_range=12., obstacle_min=.1, obstacle_max=1.2):
    if not (.02<=resolution<=.2 and 1<=maximum_range<=30 and 0<obstacle_min<obstacle_max):
        raise ValueError('invalid projection resolution/range/height')
    reduced=[]; origins=[]
    for body,pose in frames:
        # Retain actual measured endpoints only; no filling based on missing points.
        body=body[np.linalg.norm(body,axis=1)<=maximum_range]
        world=transform(body,pose)
        h=world[:,2]-(world[:,:2]@plane[:2]+plane[2])
        obstacle=(h>=obstacle_min)&(h<=obstacle_max)
        ground=(h>=-.08)&(h<=.08)
        reduced.append((world,obstacle,ground));origins.append(np.asarray(pose['xyz'])[:2])
    allxy=np.concatenate([x[0][:,:2] for x in reduced]+[np.asarray(origins)])
    low=np.floor((allxy.min(0)-.5)/resolution)*resolution
    size=np.ceil((allxy.max(0)+.5-low)/resolution).astype(int)+1
    if int(size[0])*int(size[1])>16_000_000: raise ValueError('projection exceeds grid resource limit')
    occupied=np.zeros((size[1],size[0]),bool)
    hits=np.zeros(occupied.shape,dtype=np.uint32);misses=np.zeros_like(hits)
    for origin,(world,obstacle,ground) in zip(origins,reduced):
        frame_occupied=np.zeros_like(occupied)
        obstacle_cells=np.floor((world[obstacle,:2]-low)/resolution).astype(int)
        frame_occupied[obstacle_cells[:,1],obstacle_cells[:,0]]=True
        hits+=frame_occupied
        endpoints=world[obstacle|ground,:2]
        if not len(endpoints): continue
        relative=endpoints-origin
        ranges=np.linalg.norm(relative,axis=1)
        bins=np.floor((np.arctan2(relative[:,1],relative[:,0])+np.pi)/(2*np.pi)*720).astype(int)%720
        # Nearest relevant return in each half-degree ray avoids clearing through walls.
        order=np.argsort(ranges,kind='stable')
        _,first=np.unique(bins[order],return_index=True)
        endpoints=endpoints[order[first]]
        source=np.floor((origin-low)/resolution).astype(int)
        cells=np.floor((endpoints-low)/resolution).astype(int)
        misses+=raytrace_free(frame_occupied,source,cells)
    score=.85*hits.astype(float)-.4*misses.astype(float)
    free=score<=-.4
    occupied=score>=.6
    values=np.full(occupied.shape,-1,dtype=np.int8)
    values[free]=0;values[occupied]=100
    return values,low,{'unknown':int((values==-1).sum()),'free':int((values==0).sum()),
                        'occupied':int((values==100).sum()),'evidence_fusion':{'hit_logodds':.85,'miss_logodds':-.4,'occupied_threshold':.6,'free_threshold':-.4},'sensor_origins_in_occupied_cells':sum(
                            bool(occupied[tuple(np.floor((o-low)/resolution).astype(int)[::-1])]) for o in origins)}
