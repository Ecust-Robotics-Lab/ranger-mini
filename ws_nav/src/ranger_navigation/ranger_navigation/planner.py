"""SE(2) hybrid lattice with rectangle/cell SAT and swept primitive validation."""
from dataclasses import dataclass
import heapq
import itertools
import math
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from .geometry import Body, wrap, integrate, densify, validate_sweep
from .models import Mode, Limits
from .coverage_policy import CoveragePolicy


class NoPath(RuntimeError):
    def __init__(self,reason,details=None):
        super().__init__(reason)
        self.details = details or {}


class Grid:
    @classmethod
    def from_yaml(cls,path):
        from pathlib import Path
        import yaml
        from PIL import Image
        path = Path(path)
        config = yaml.safe_load(path.read_text())
        image = np.flipud(np.array(Image.open(path.parent/config['image']).convert('L')))
        probability = image/255. if config.get('negate',0) else 1-image/255.
        cells = np.full(image.shape,-1,dtype=np.int16)
        cells[probability<float(config['free_thresh'])] = 0
        cells[probability>float(config['occupied_thresh'])] = 100
        return cls(cells,config['resolution'],config['origin'])

    def __init__(self, cells, resolution, origin=(0.,0.,0.)):
        self.cells = np.asarray(cells, dtype=np.int16).copy()
        self.resolution = float(resolution)
        self.origin = np.asarray(origin,dtype=float)
        if (self.cells.ndim!=2 or not self.cells.size or self.resolution<=0
                or not np.isfinite(self.origin).all() or abs(self.origin[2])>1e-8):
            raise ValueError('invalid/rotated grid')
        self.blocked = (self.cells<0) | (self.cells>=50)
        self.distance = distance_transform_edt(~self.blocked)*self.resolution
        self.local_visibility = None
        self.guidance_visibility_reserve = .10
        self.coverage_policy=CoveragePolicy()
        self.dynamic_points,self.dynamic_tree = np.empty((0,2)),None

    def with_obstacles(self, points):
        # Retain the observed voxel centers as a temporary occupancy layer.
        # Re-rasterizing them on a second grid and adding point uncertainty
        # again can reject a pose that the independent controller just verified.
        layer = Grid(self.cells,self.resolution,self.origin)
        layer.dynamic_points = np.asarray(points,dtype=float).reshape(-1,2).copy()
        if len(layer.dynamic_points):
            layer.dynamic_tree = cKDTree(layer.dynamic_points)
        return layer

    def squares_free(self,centers,yaw,half=.025):
        """Vectorized exact cell SAT, for reprojecting observed free cells."""
        centers = np.asarray(centers).reshape(-1,2)
        mid = np.floor((centers-self.origin[:2])/self.resolution).astype(int)
        radius = math.ceil(math.sqrt(2)*half/self.resolution)+1
        a,b = np.meshgrid(np.arange(-radius,radius+1),np.arange(-radius,radius+1))
        ix,iy = mid[:,0,None]+a.ravel(),mid[:,1,None]+b.ravel()
        h,w = self.cells.shape
        blocked = (ix<0)|(ix>=w)|(iy<0)|(iy>=h)|self.blocked[np.clip(iy,0,h-1),np.clip(ix,0,w-1)]
        dx = (ix+.5)*self.resolution+self.origin[0]-centers[:,0,None]
        dy = (iy+.5)*self.resolution+self.origin[1]-centers[:,1,None]
        c,s = math.cos(yaw),math.sin(yaw)
        r,extent = self.resolution/2,half*(abs(c)+abs(s))
        overlap = ((np.abs(dx)<=extent+r)&(np.abs(dy)<=extent+r)
                   &(np.abs(c*dx+s*dy)<=half+r*(abs(c)+abs(s)))
                   &(np.abs(-s*dx+c*dy)<=half+r*(abs(c)+abs(s))))
        return ~np.any(blocked&overlap,axis=1)

    def free(self, pose, body=Body(), margin=None):
        if not self._free_cells(pose,body,margin):
            return False
        if self.local_visibility is not None:
            visible,observer,start,radius = self.local_visibility
            if np.linalg.norm(np.asarray(pose)[:2]-start[:2])<=radius:
                c,s = math.cos(observer[2]),math.sin(observer[2])
                xy = (np.asarray(pose)[:2]-observer[:2])@np.array([[c,-s],[s,c]])
                # Reserve observed space around guidance for sensor viewpoint
                # changes while executing/stopping, beyond physical clearance.
                return visible.covered([*xy,pose[2]-observer[2]],body,self.coverage_policy)
        return True

    def _free_cells(self, pose, body=Body(), margin=None,check_dynamic=True):
        margin = body.clearance if margin is None else margin
        x,y,yaw = pose
        if check_dynamic and self.dynamic_tree is not None:
            point_margin = body.clearance+.04+.015
            near = self.dynamic_tree.query_ball_point([x,y],body.radius+point_margin)
            if near and np.min(body.distances(pose,self.dynamic_points[near]))<point_margin:
                return False
        half = body.half+margin
        c,s = abs(math.cos(yaw)),abs(math.sin(yaw))
        ext = np.array([c*half[0]+s*half[1],s*half[0]+c*half[1]])
        lo = np.floor((np.array([x,y])-ext-self.origin[:2])/self.resolution).astype(int)
        hi = np.floor((np.array([x,y])+ext-self.origin[:2])/self.resolution).astype(int)
        h,w = self.cells.shape
        if np.any(lo<0) or hi[0]>=w or hi[1]>=h:
            return False
        mid = np.floor((np.array([x,y])-self.origin[:2])/self.resolution).astype(int)
        if self.distance[mid[1],mid[0]] > np.linalg.norm(half)+1.5*self.resolution:
            return True
        iy,ix = np.nonzero(self.blocked[lo[1]:hi[1]+1,lo[0]:hi[0]+1])
        if not len(ix):
            return True
        dx = (ix+lo[0]+.5)*self.resolution+self.origin[0]-x
        dy = (iy+lo[1]+.5)*self.resolution+self.origin[1]-y
        c,s = math.cos(yaw),math.sin(yaw)
        r = self.resolution/2
        # Four separating axes (world x/y already bounded above, test again
        # to exclude boundary-only bbox cells; body x/y complete the SAT).
        overlap = ((np.abs(dx)<=ext[0]+r)&(np.abs(dy)<=ext[1]+r)
                   &(np.abs(c*dx+s*dy)<=half[0]+r*(abs(c)+abs(s)))
                   &(np.abs(-s*dx+c*dy)<=half[1]+r*(abs(c)+abs(s))))
        return not np.any(overlap)

    def coverage_report(self,pose,body,policy):
        """Check separate interior and edge allowances for the observed grid.

        This does not modify the grid or make an unknown global-map cell free.
        With boundary tolerance disabled, any protected intersection counts
        toward the protected allowance. Known obstacles always block.
        """
        def intersected(margin,contained=False):
            x,y,yaw=pose;half=body.half+margin;c,s=math.cos(yaw),math.sin(yaw)
            extent=np.array([abs(c)*half[0]+abs(s)*half[1],abs(s)*half[0]+abs(c)*half[1]])
            lo=np.floor((np.array([x,y])-extent-self.origin[:2])/self.resolution).astype(int)
            hi=np.floor((np.array([x,y])+extent-self.origin[:2])/self.resolution).astype(int)
            h,w=self.cells.shape
            if np.any(lo<0) or hi[0]>=w or hi[1]>=h:return None
            iy,ix=np.nonzero(self.blocked[lo[1]:hi[1]+1,lo[0]:hi[0]+1])
            dx=(ix+lo[0]+.5)*self.resolution+self.origin[0]-x
            dy=(iy+lo[1]+.5)*self.resolution+self.origin[1]-y;r=self.resolution/2
            overlap=((np.abs(dx)<=extent[0]+r)&(np.abs(dy)<=extent[1]+r)&
                     (np.abs(c*dx+s*dy)<=half[0]+r*(abs(c)+abs(s)))&
                     (np.abs(-s*dx+c*dy)<=half[1]+r*(abs(c)+abs(s))))
            if contained:
                # A 5cm cell touching the rectangle's edge is a boundary
                # uncertainty, not a whole missing interior observation.
                overlap &= ((np.abs(c*dx+s*dy)+r*(abs(c)+abs(s))<=half[0])&
                            (np.abs(-s*dx+c*dy)+r*(abs(c)+abs(s))<=half[1]))
            return self.cells[iy[overlap]+lo[1],ix[overlap]+lo[0]]
        report=dict(accepted=False,total_margin_m=policy.total_margin,protected_margin_m=policy.protected_margin,
                    unknown_cell_budget=policy.unknown_cell_budget,allow_boundary_unknowns=policy.allow_boundary_unknowns,
                    protected_unknown_cell_budget=policy.protected_unknown_cell_budget,
                    unknown_edge_cells=0,unknown_protected_cells=0,reason='coverage_outside_grid')
        outer=intersected(policy.total_margin)
        if outer is None:return report
        if np.any(outer>=50):report['reason']='coverage_known_obstacle';return report
        inner=intersected(policy.protected_margin,contained=policy.allow_boundary_unknowns)
        if inner is None:return report
        protected = int(np.count_nonzero(inner < 0))
        edge = int(np.count_nonzero(outer < 0)) - protected
        report.update(unknown_edge_cells=edge,unknown_protected_cells=protected)
        report['reason']=('coverage_body_unobserved' if protected > policy.protected_unknown_cell_budget else
                          'coverage_edge_budget_exceeded' if edge > policy.unknown_cell_budget else 'coverage_valid')
        report['accepted']=report['reason']=='coverage_valid'
        return report

    def covered(self,pose,body,policy):
        return self.coverage_report(pose,body,policy)['accepted']


@dataclass
class Segment:
    mode: Mode
    poses: np.ndarray
    reverse: bool = False


def reanchor_parallel(segment,pose,body,grid,points,static_reserve,static_clearance=None):
    """Choose a fixed wheel axis at rest and independently validate its sweep.

    Localization noise must not force a whole new search while steering is
    stopped. This changes the proposed axis before execution, never a moving
    command or an already validated control result.
    """
    if segment.mode!=Mode.PARALLEL:
        return None
    if not parallel_anchor_close(segment,pose):
        return None  # appreciable displacement requires a new route
    target=segment.poses[-1].copy()
    target[2]=pose[2]
    path=np.array([pose,target])
    okay,_,_=validate_sweep(path,points,body,grid=grid,
                           margin=body.clearance+.04,
                           map_margin=body.clearance+static_reserve if static_clearance is None else static_clearance)
    return Segment(Mode.PARALLEL,path,segment.reverse) if okay else None


def parallel_anchor_close(segment,pose):
    error=np.asarray(pose)-segment.poses[0]
    axis=segment.poses[-1,:2]-segment.poses[0,:2]
    length=np.linalg.norm(axis)
    if length<1e-7:
        return np.linalg.norm(error[:2])<=.03 and abs(wrap(error[2]))<=.03
    axis=axis/length
    along=float(np.dot(error[:2],axis))
    transverse=error[:2]-along*axis
    return (np.linalg.norm(transverse)<=.03 and -.03<=along<=length+.03
            and abs(wrap(error[2]))<=.03)


class Planner:
    def __init__(self, body=Body(), limits=Limits(), bins=32, step=.15,
                 max_expansions=50000, timeout=5., static_reserve=.04, prefer_parallel_in_narrow=False,static_clearance=None):
        self.body,self.limits = body,limits
        self.bins,self.step = bins,step
        self.max_expansions,self.timeout = max_expansions,timeout
        if not np.isfinite(static_reserve) or not 0 <= static_reserve <= .10:
            raise ValueError('static map reserve must be finite and in [0, .10]m')
        self.static_reserve = float(static_reserve)
        self.map_clearance = self.body.clearance+self.static_reserve if static_clearance is None else float(static_clearance)
        if not np.isfinite(self.map_clearance) or not 0<=self.map_clearance<=.2:
            raise ValueError('static clearance must be finite and in [0, .2]m')
        self.prefer_parallel_in_narrow = bool(prefer_parallel_in_narrow)
        self.expansions = 0

    def key(self,p,mode):
        return (round(p[0]/.075),round(p[1]/.075),
                round(float(wrap(p[2]))*self.bins/(2*np.pi))%self.bins,int(mode))

    def edge_free(self, grid, poses):
        # Cover inter-sample footprint displacement conservatively.
        step = min(.015,grid.resolution/3)
        # Static map reserve is configurable independently of point uncertainty;
        # dynamic voxel uncertainty is checked once in the separate point layer.
        return all(grid.free(p,self.body,self.map_clearance+step)
                   for p in densify(poses,self.body,step))

    def primitives(self,p):
        da = 2*np.pi/self.bins
        # Constant-curvature arcs; reversing preserves the sign of body yaw.
        for sign in (1,-1):
            for curvature in (0.,1/self.limits.min_radius,-1/self.limits.min_radius):
                velocity = [sign,0,sign*curvature]
                n = max(2,math.ceil(self.step/.025))
                edge = np.asarray([integrate(p,velocity,t*self.step/n) for t in range(n+1)])
                yield Mode.DRIVE,edge,sign<0,self.step*(1.1 if sign<0 else 1.)
        # Fixed heading translation in eight body directions, including pure lateral.
        for a in np.arange(8)*np.pi/4:
            edge = np.array([p,integrate(p,[math.cos(a),math.sin(a),0],self.step)])
            yield Mode.PARALLEL,edge,False,self.step*1.08
        for sign in (1,-1):
            edge = np.array([p,integrate(p,[0,0,sign],da)])
            yield Mode.SPIN,edge,False,da*.30

    def plan(self,grid,start,goal,start_mode=Mode.DRIVE):
        start,goal = np.array(start,dtype=float),np.array(goal,dtype=float)
        if not np.isfinite([start,goal]).all():
            raise NoPath('nonfinite_start_or_goal')
        margin = self.map_clearance+.015
        endpoints = {}
        for name,pose in (('start',start),('goal',goal)):
            physical_static = grid._free_cells(pose,self.body,0.,check_dynamic=False)
            static = grid._free_cells(pose,self.body,margin,check_dynamic=False)
            dynamic = grid.dynamic_tree is None or not any(
                self.body.distances(pose,grid.dynamic_points)<self.body.clearance+.04+.015)
            free = grid.free(pose,self.body,margin)
            reason = ('static_collision_or_unknown' if not physical_static else
                      'static_clearance_insufficient' if not static else
                      'live_obstacle_collision' if not dynamic else
                      'unobserved' if not free else None)
            endpoints[name] = dict(pose=pose.tolist(),free=bool(free),static_free=bool(static),
                physical_static_free=bool(physical_static),dynamic_free=bool(dynamic),
                static_margin_m=margin,reason=(name+'_'+reason if reason else None))
        if not all(x['free'] for x in endpoints.values()):
            raise NoPath(next(x['reason'] for x in endpoints.values() if not x['free']),endpoints)
        # An exact, swept connection is independent of lattice length/yaw bins.
        # A clear 30cm fixed-heading request needs one fixed steering axis,
        # rather than a drive primitive followed by a short side-step/spin.
        at=np.array([*goal[:2],start[2]])
        if self.edge_free(grid,[start,at]) and self.edge_free(grid,[at,goal]):
            direct=[]
            if np.linalg.norm(at[:2]-start[:2])>1e-7:
                direct.append(Segment(Mode.PARALLEL,np.array([start,at])))
            if abs(wrap(goal[2]-start[2]))>1e-7:
                direct.append(Segment(Mode.SPIN,np.array([at,goal])))
            self.expansions=0
            return direct
        counter = itertools.count()
        def heuristic(p):
            return float(np.linalg.norm(p[:2]-goal[:2]))
        # Legacy side-slip feedback can be present at startup. It is not a
        # prediction model: search a supported route, then the task must stop
        # and confirm the route's explicit mode before executing any command.
        start_mode = Mode(start_mode) if start_mode in (0,1,2) else Mode.DRIVE
        k0 = self.key(start,start_mode)
        # Immutable nodes: replacing a cheaper lattice representative must not
        # move a predecessor beneath an already-created child trajectory.
        queue = [(heuristic(start),0,0)]
        states = [(start,start_mode,False)]
        costs,parents = [0.],{}
        best = {k0:0}
        self.expansions = 0
        began = time.monotonic()
        final = None
        while queue:
            if self.expansions>=self.max_expansions or time.monotonic()-began>self.timeout:
                raise NoPath('search_budget_exceeded')
            _,_,key = heapq.heappop(queue)
            p,previous_mode,previous_reverse = states[key]
            if best.get(self.key(p,previous_mode))!=key:
                continue
            self.expansions += 1
            # Exact final connection: translation at fixed heading, then swept spin.
            if heuristic(p)<=self.step*1.8:
                at = np.array([*goal[:2],p[2]])
                if self.edge_free(grid,[p,at]) and self.edge_free(grid,[at,goal]):
                    final = (key,at)
                    break
            for mode,edge,reverse,cost in self.primitives(p):
                q = edge[-1]
                if self.prefer_parallel_in_narrow and mode==Mode.DRIVE:
                    ix,iy = np.floor((q[:2]-grid.origin[:2])/grid.resolution).astype(int)
                    h,w = grid.cells.shape
                    if (0<=ix<w and 0<=iy<h and
                            grid.distance[iy,ix]<self.body.radius+self.map_clearance+.015):
                        # Fixed-steering translation holds body yaw in a narrow
                        # straight passage. Curved drive is still available,
                        # but has higher cost where small yaw errors use up
                        # the available side clearance.
                        cost *= 3.
                nk = self.key(q,mode)
                switching = mode!=previous_mode or (mode==Mode.DRIVE and reverse!=previous_reverse)
                score = costs[key]+cost+(.22 if switching else 0.)
                if (nk in best and score>=costs[best[nk]]) or not self.edge_free(grid,edge):
                    continue
                node = len(states)
                costs.append(score)
                states.append((q,mode,reverse))
                parents[node],best[nk] = (key,edge),node
                heapq.heappush(queue,(score+1.5*heuristic(q),next(counter),node))
        if final is None:
            raise NoPath('no_route')
        key,at = final
        tail = [(Mode.PARALLEL,np.array([states[key][0],at]),False),
                (Mode.SPIN,np.array([at,goal]),False)]
        edges = []
        while key!=0:
            old,edge = parents[key]
            _,mode,reverse = states[key]
            edges.append((mode,edge,reverse))
            key = old
        edges = edges[::-1]+tail
        segments = []
        for mode,edge,reverse in edges:
            if np.linalg.norm(edge[-1,:2]-edge[0,:2])+abs(wrap(edge[-1,2]-edge[0,2]))<1e-7:
                continue
            same = segments and (segments[-1].mode,segments[-1].reverse)==(mode,reverse)
            if same and mode==Mode.PARALLEL:
                previous = segments[-1].poses[-1,:2]-segments[-1].poses[0,:2]
                current = edge[-1,:2]-edge[0,:2]
                same = np.dot(previous,current)>np.linalg.norm(previous)*np.linalg.norm(current)*(1-1e-6)
            if same and mode==Mode.DRIVE:
                # A drive segment has one physical steering position. A new
                # curvature starts a stopped segment, even in the same mode.
                def curvature(poses):
                    distance=np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1).sum()
                    return float(wrap(poses[-1,2]-poses[0,2]))/max(distance,1e-9)
                same = abs(curvature(segments[-1].poses)-curvature(edge))<1e-5
            if same:
                segments[-1].poses = np.vstack((segments[-1].poses,edge[1:]))
            else:
                segments.append(Segment(mode,np.asarray(edge),reverse))
        return segments
