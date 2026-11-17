"""Sample-time 3D filtering, spatial sampling, visibility and bounded memory.

2D visibility uses actual finite rays clipped to the obstacle height band;
nearest obstacles bound clearing. Empty sectors/clouds are never free space.
"""
import math
import numpy as np
from .geometry import Body
from .planner import Grid


class ObstacleMemory:
    def __init__(self, body=Body(), voxel=.05, sectors=180, retention=2.,
                 max_cells=30000, min_height=.10, max_height=.80, max_range=8., angular_guard=1, control_memory=0.):
        if sectors<90 or sectors>720 or angular_guard not in (0,1):
            raise ValueError('invalid visibility angular configuration')
        self.body,self.voxel,self.sectors = body,voxel,sectors
        if not np.isfinite(control_memory) or not 0 <= control_memory <= min(2.,retention):
            raise ValueError("control memory must not exceed obstacle retention or 2 seconds")
        self.control_memory = float(control_memory)
        self.angular_guard = angular_guard
        self.retention,self.max_cells = retention,max_cells
        self.min_height,self.max_height,self.max_range = min_height,max_height,max_range
        self.reset()

    def reset(self):
        self.cells = {}
        self.last_stamp = None
        self.pose = None
        self.visibility_pose = None
        self.ranges = np.zeros(self.sectors)
        self.error = 'no_observation'
        self._visible_grid = None
        self.control_grid = None
        self._visibility_history = []

    def bins(self, xy):
        return np.floor((np.arctan2(xy[:,1],xy[:,0])+np.pi)*self.sectors/(2*np.pi)).astype(int)%self.sectors

    def update(self, xyz_imu, r_base_imu, t_base_imu, pose_odom_base, stamp):
        if self.last_stamp is not None and stamp<=self.last_stamp:
            self.error = 'cloud_time_regression'
            return False
        xyz = np.asarray(xyz_imu,dtype=float).reshape(-1,3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            self.error = 'empty_cloud'
            return False
        xyz = xyz @ np.asarray(r_base_imu).T + t_base_imu
        self_mask = ((np.abs(xyz[:,0])<=self.body.half[0])
                     &(np.abs(xyz[:,1])<=self.body.half[1])&(xyz[:,2]<=.75))
        ranges = np.linalg.norm(xyz[:,:2],axis=1)
        keep = (~self_mask)&(xyz[:,2]>=self.min_height)&(xyz[:,2]<=self.max_height)&(ranges<=self.max_range)
        xy = xyz[keep,:2]
        sensor_xy = np.asarray(self.body.sensor_origin_xy)
        # A floor/ceiling endpoint is not an obstacle in the selected band, but
        # its ray still observes part of that band. Clip at the height crossing
        # instead of discarding all free-space evidence from such returns.
        ray_points = xyz[~self_mask]
        z0 = self.body.sensor_origin_z
        dz = ray_points[:,2]-z0
        fraction = np.ones(len(ray_points))
        low,high = ray_points[:,2]<self.min_height,ray_points[:,2]>self.max_height
        fraction[low] = (self.min_height-z0)/dz[low]
        fraction[high] = (self.max_height-z0)/dz[high]
        ray_xy = ray_points[:,:2]-sensor_xy
        observed = np.zeros(self.sectors)
        finite_lengths = np.minimum(np.linalg.norm(ray_xy,axis=1)*np.clip(fraction,0.,1.),self.max_range)
        np.maximum.at(observed,self.bins(ray_xy),finite_lengths)
        nearest = np.full(self.sectors,np.inf)
        rays = xy-sensor_xy
        np.minimum.at(nearest,self.bins(rays),np.linalg.norm(rays,axis=1))
        observed = np.minimum(observed,nearest)
        if not np.any(observed>0):
            self.error = 'no_height_band_rays'
            return False
        p = np.asarray(pose_odom_base,dtype=float)
        c,s = math.cos(p[2]),math.sin(p[2])
        rot = np.array([[c,-s],[s,c]])
        # Delete remembered cells only when a farther observation verifies the
        # corresponding ray, or when memory expires (expired space is unknown).
        if self.cells:
            keys = list(self.cells)
            old = (np.asarray(keys)+.5)*self.voxel
            rel = (old-p[:2])@rot
            rays = rel-sensor_xy
            rr = np.linalg.norm(rays,axis=1)
            sector = self.bins(rays)
            # Neighbour sectors must agree; no clearing across sparse beams.
            visible = np.minimum.reduce([observed, np.roll(observed,1),np.roll(observed,-1)])
            for i,k in enumerate(keys):
                if (stamp-self.cells[k]>self.retention or
                        rr[i]+2*self.voxel<visible[sector[i]]):
                    del self.cells[k]
        world = xy@rot.T+p[:2]
        for key in np.unique(np.floor(world/self.voxel).astype(int),axis=0):
            self.cells[tuple(key)] = stamp
        if len(self.cells)>self.max_cells:
            self.error = 'obstacle_capacity_exceeded'
            return False
        self.last_stamp,self.pose,self.ranges,self.error = stamp,p,observed,None
        self._build_visibility_grid(retain=True)
        return True

    def points(self):
        return (np.asarray(list(self.cells),dtype=float).reshape(-1,2)+.5)*self.voxel

    def sampled(self, max_points):
        """Round-robin angular/radial strata; never nearest-N truncation.

        The complete voxel set remains available for final verification. The
        optimization margin must additionally cover the voxel half diagonal.
        """
        return self._sample_points(self.points(),max_points)

    def _sample_points(self,pts,max_points):
        if len(pts)<=max_points:
            return pts
        rel = pts-self.pose[:2]
        bins = self.bins(rel)
        radii = np.linalg.norm(rel,axis=1)
        groups = [np.where(bins==b)[0] for b in range(self.sectors)]
        groups = [g[np.argsort(radii[g])] for g in groups if len(g)]
        out = []
        level = 0
        while len(out)<max_points:
            for g in groups:
                if level<len(g):
                    out.append(g[level])
                    if len(out)==max_points:
                        break
            level += 1
        return pts[out]

    def covers(self, pose, body, margin):
        if self.error or self.pose is None:
            return False
        if self._visible_grid is None:
            self._build_visibility_grid()
        observer = self.visibility_pose
        c,s = math.cos(observer[2]),math.sin(observer[2])
        rel = (np.asarray(pose[:2])-observer[:2])@np.array([[c,-s],[s,c]])
        # Grid SAT checks every cell intersected by the *whole* rectangle.
        # The physical margin is applied once, avoiding double voxel inflation.
        return self._visible_grid.free([*rel,pose[2]-observer[2]],body,margin)

    def set_visibility(self,pose,ranges,cells=None):
        self.pose,self.ranges,self.error = np.asarray(pose),np.asarray(ranges),None
        self.visibility_pose = self.pose.copy()
        if cells is not None:
            self._visible_grid = Grid(np.asarray(cells).reshape(80,80),.05,(-2.,-2.,0.))
        else:
            self._build_visibility_grid()

    def _build_visibility_grid(self,retain=False):
        resolution,extent = .05,2.
        axis = np.arange(-extent,extent,resolution)
        x,y = np.meshgrid(axis,axis)
        corners = np.stack([np.stack((x+dx,y+dy),axis=-1) for dx,dy in
                            ((0.,0.),(resolution,0.),(0.,resolution),(resolution,resolution))],axis=2)
        # Keep cell boundaries fixed in odom while the robot translates/turns.
        # Repeated rasterization in rotating body grids otherwise erodes known
        # free cells and can strand a previously verified stopped footprint.
        self.visibility_pose = np.r_[np.floor(self.pose[:2]/resolution)*resolution,0.]
        c,s = math.cos(self.pose[2]),math.sin(self.pose[2])
        body_corners = (corners+self.visibility_pose[:2]-self.pose[:2])@np.array([[c,-s],[s,c]])
        rays = body_corners-np.asarray(self.body.sensor_origin_xy)
        bins = self.bins(rays.reshape(-1,2)).reshape(rays.shape[:-1])
        if self.angular_guard:
            visible = np.minimum.reduce([self.ranges,np.roll(self.ranges,1),np.roll(self.ranges,-1)])
            observed = np.all(np.linalg.norm(rays,axis=-1)<visible[bins]-.01,axis=2)
        else:
            # Check every sector intersecting each cell, including sectors
            # between its corners. Avoid expanding a short return into the
            # two adjacent sectors when finer angular sampling is selected.
            minimum = sector_minimum(rays,self.ranges)
            observed = np.linalg.norm(rays,axis=-1).max(2)<minimum-.01
        known_body = np.all(np.all(np.abs(body_corners)<=self.body.half,axis=-1),axis=2)
        cells = np.where(observed|known_body,0,-1)
        current = Grid(cells,resolution,(-extent,-extent,0.))
        # The optional short control memory uses raw sample timestamps only;
        # a union never refreshes its own age. New occlusion is applied below.
        control_cells = cells.copy()
        if retain and self.last_stamp is not None:
            self._visibility_history = [f for f in self._visibility_history if 0<=self.last_stamp-f[0]<=self.retention][-31:]
            # Reproject only cells whose entire area was actually observed.
            # Raw frames retain original sample times; a derived union is never
            # stored back into history or allowed to refresh its own expiry.
            for frame_stamp,observer,old in self._visibility_history:
                iy,ix = np.nonzero((cells<0)|(control_cells<0))
                if not len(ix):
                    break
                local = np.column_stack((ix,iy))*resolution+np.array([-extent+resolution/2]*2)
                c,s = math.cos(self.visibility_pose[2]),math.sin(self.visibility_pose[2])
                world = local@np.array([[c,s],[-s,c]])+self.visibility_pose[:2]
                c,s = math.cos(observer[2]),math.sin(observer[2])
                relative = (world-observer[:2])@np.array([[c,-s],[s,c]])
                # All stored grids share odom-aligned cell boundaries. Integer
                # index translation avoids boundary-roundoff erosion in SAT.
                old_indices = np.rint((relative-old.origin[:2])/resolution-.5).astype(int)
                valid = (old_indices[:,0]>=0)&(old_indices[:,0]<80)&(old_indices[:,1]>=0)&(old_indices[:,1]<80)
                known = np.zeros(len(relative),dtype=bool)
                known[valid] = old.cells[old_indices[valid,1],old_indices[valid,0]]==0
                cells[iy[known],ix[known]] = 0
                if self.control_memory>0 and self.last_stamp-frame_stamp<=self.control_memory:
                    control_cells[iy[known],ix[known]] = 0
            self._visibility_history.append((self.last_stamp,self.visibility_pose.copy(),current))
        # A newly observed/remembered obstacle immediately revokes old free
        # evidence behind it. Dilate each voxel's angular footprint; checking
        # every crossed sector also handles cells straddling the ray origin.
        points = self.points()
        if len(points):
            # c,s may have been reused for history reprojection (yaw zero).
            cb,sb=math.cos(self.pose[2]),math.sin(self.pose[2])
            relative=(points-self.pose[:2])@np.array([[cb,-sb],[sb,cb]])-np.asarray(self.body.sensor_origin_xy)
            radius=np.linalg.norm(relative,axis=1)
            nearest=np.full(self.sectors,np.inf)
            sectors=self.bins(relative)
            spread=np.ceil(np.arctan2(self.voxel*.71,np.maximum(radius,1e-6))*self.sectors/(2*np.pi)).astype(int)+1
            for offset in range(-int(spread.max()),int(spread.max())+1):
                valid=abs(offset)<=spread
                np.minimum.at(nearest,(sectors[valid]+offset)%self.sectors,np.maximum(0.,radius[valid]-self.voxel*.71))
            shadow=np.linalg.norm(rays,axis=-1).max(2)>=sector_minimum(rays,nearest)
            control_cells[shadow & ~known_body]=-1
            cells[shadow & ~known_body]=-1
        self.control_grid=Grid(control_cells,resolution,(-extent,-extent,0.))
        self._visible_grid = Grid(cells,resolution,(-extent,-extent,0.))


def sector_minimum(rays,ranges):
    sectors=len(ranges)
    center = np.arctan2(rays[...,1].mean(2),rays[...,0].mean(2))
    angle = np.arctan2(rays[...,1],rays[...,0])
    delta = (angle-center[...,None]+np.pi)%(2*np.pi)-np.pi
    first = np.floor((center+delta.min(2)+np.pi)*sectors/(2*np.pi)).astype(int)
    last = np.floor((center+delta.max(2)+np.pi)*sectors/(2*np.pi)).astype(int)
    count = np.minimum(last-first+1,sectors)
    inside = ((rays[...,0].min(2)<=0)&(rays[...,0].max(2)>=0)
              &(rays[...,1].min(2)<=0)&(rays[...,1].max(2)>=0))
    count = np.where(inside,sectors,count)
    # Circular sparse table: exact range minimum in constant time per
    # cell after O(sectors*log(sectors)) preprocessing.
    levels = [ranges]
    for k in range(1,int(np.floor(np.log2(sectors)))+1):
        levels.append(np.minimum(levels[-1],np.roll(levels[-1],-(1<<(k-1)))))
    table = np.asarray(levels)
    k = np.floor(np.log2(count)).astype(int)
    minimum = np.minimum(table[k,first%sectors],table[k,(first+count-(1<<k))%sectors])
    return minimum
