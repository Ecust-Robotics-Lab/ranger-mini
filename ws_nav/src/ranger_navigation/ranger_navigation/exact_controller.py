"""GPL-3.0: V2 adaptation of the pinned EXACT-MPPI optimizer.

Reuses upstream noise generation, weighted updates, critics, SDF and validator.
Every rollout and the final weighted sequence use the same mode constraints;
the upstream post-validation Savitzky-Golay filter is deliberately not invoked.
"""
from dataclasses import replace,dataclass,fields
from functools import partial
import time
import math
import numpy as np
import jax
import jax.numpy as jnp
from exact_mppi.mppi_jax.optimizer import Optimizer
from exact_mppi.mppi_jax.models import ControlSequence, Rectangles, Trajectories
from exact_mppi.mppi_jax.models.state import State
from exact_mppi.mppi_jax.optimal_trajectory_validator import ValidationResult
from .geometry import Body, wrap, densify
from .models import Mode, Limits, feasible, rollout, braking_commands


@jax.tree_util.register_dataclass
@dataclass(frozen=True,slots=True)
class V2State(State):
    direction: jax.Array
    coverage_cells: jax.Array
    coverage_observer: jax.Array
    coverage_enabled: jax.Array
    reference_controls: jax.Array
    drive_constraint: jax.Array


class VisibilityCritics:
    """Keep raw-point EXACT costs; add the observed-space feasibility cost."""
    def __init__(self,upstream,body,visibility_reserve=.10):
        self.upstream = upstream
        self.half = jnp.asarray(body.half+body.clearance+.015,dtype=jnp.float32)
        self.reserve_half = jnp.asarray(body.half+body.clearance+visibility_reserve,dtype=jnp.float32)
        radius = math.ceil(float(np.linalg.norm(np.asarray(self.reserve_half)))/.05)+1
        x,y = np.meshgrid(np.arange(-radius,radius+1),np.arange(-radius,radius+1))
        self.offsets = jnp.asarray(np.column_stack((x.ravel(),y.ravel())),dtype=jnp.int32)

    def evalTrajectoriesScores(self,data):
        costs,fail,info = self.upstream.evalTrajectoriesScores(data)
        def visibility(_):
            tr,observer = data.trajectories,data.state.coverage_observer
            c,s = jnp.cos(observer[2]),jnp.sin(observer[2])
            dx,dy = tr.x-observer[0],tr.y-observer[1]
            x,y = c*dx+s*dy,-s*dx+c*dy
            yaw = tr.yaws-observer[2]
            ix = jnp.floor((x+2.)/.05).astype(jnp.int32)[...,None]+self.offsets[:,0]
            iy = jnp.floor((y+2.)/.05).astype(jnp.int32)[...,None]+self.offsets[:,1]
            unknown = (ix<0)|(ix>=80)|(iy<0)|(iy>=80)|(data.state.coverage_cells[jnp.clip(iy,0,79),jnp.clip(ix,0,79)]<0)
            dx,dy = (ix+.5)*.05-2.-x[...,None],(iy+.5)*.05-2.-y[...,None]
            c,s = jnp.cos(yaw)[...,None],jnp.sin(yaw)[...,None]
            ac,ass = jnp.abs(c),jnp.abs(s)
            # Same four separating axes as the independent NumPy Grid checker.
            overlap = ((jnp.abs(dx)<=ac*self.half[0]+ass*self.half[1]+.025)
                       &(jnp.abs(dy)<=ass*self.half[0]+ac*self.half[1]+.025)
                       &(jnp.abs(c*dx+s*dy)<=self.half[0]+.025*(ac+ass))
                       &(jnp.abs(-s*dx+c*dy)<=self.half[1]+.025*(ac+ass)))
            collided = jnp.any(unknown&overlap,axis=(1,2))
            reserve = ((jnp.abs(dx)<=ac*self.reserve_half[0]+ass*self.reserve_half[1]+.025)
                       &(jnp.abs(dy)<=ass*self.reserve_half[0]+ac*self.reserve_half[1]+.025)
                       &(jnp.abs(c*dx+s*dy)<=self.reserve_half[0]+.025*(ac+ass))
                       &(jnp.abs(-s*dx+c*dy)<=self.reserve_half[1]+.025*(ac+ass)))
            near_unknown = jnp.mean(jnp.any(unknown&reserve,axis=2),axis=1)
            return jnp.where(collided,1e6,120.*near_unknown),jnp.all(collided)
        extra,all_blocked = jax.lax.cond(data.state.coverage_enabled,visibility,
                                        lambda _: (jnp.zeros_like(costs),jnp.array(False)),None)
        return costs+extra,fail|all_blocked,info


@partial(jax.jit,static_argnames=('mode','limits','dt'))
def _project(controls, initial, mode, limits, dt, direction, drive_constraint=None):
    """Projection occurs inside candidate prediction and before scoring."""
    braking_steps = int(np.ceil(max(limits.speed/limits.acceleration,
                                    limits.yaw_rate/limits.yaw_acceleration)/dt))
    # Every candidate contains its own bounded stop. This keeps a finite-horizon
    # solution from ending at an observed-space boundary with nonzero speed.
    controls = controls.at[-braking_steps:].set(jnp.zeros_like(controls[-braking_steps:]))
    def step(old, requested):
        delta = requested[:2]-old[:2]
        norm = jnp.linalg.norm(delta)
        v = old[:2]+delta*jnp.minimum(1.,limits.acceleration*dt/jnp.maximum(norm,1e-9))
        v *= jnp.minimum(1.,limits.speed/jnp.maximum(jnp.linalg.norm(v),1e-9))
        w = jnp.clip(requested[2],old[2]-limits.yaw_acceleration*dt,old[2]+limits.yaw_acceleration*dt)
        w = jnp.clip(w,-limits.yaw_rate,limits.yaw_rate)
        if mode == Mode.DRIVE:
            v = v.at[1].set(0.)
            if drive_constraint is None:
                minimum_v = limits.min_radius*jnp.maximum(0.,jnp.abs(old[2])-limits.yaw_acceleration*dt)
                sign = jnp.where(old[0]>=0.,1.,-1.)
                vx = jnp.where(minimum_v>0.,sign*jnp.maximum(sign*v[0],minimum_v),v[0])
                v = v.at[0].set(vx)
                w = jnp.clip(w,-jnp.abs(v[0])/limits.min_radius,jnp.abs(v[0])/limits.min_radius)
            else:
                # Optimize speed on the verified constant-curvature arc.
                # Independent noisy vx/wz near zero must not request alternating
                # full-lock steering. This constraint is inside every rollout.
                curvature,sign=drive_constraint
                nonzero=jnp.abs(curvature)>1e-6
                denominator=jnp.where(nonzero,curvature,1.)
                a=(old[2]-limits.yaw_acceleration*dt)/denominator
                b=(old[2]+limits.yaw_acceleration*dt)/denominator
                lo=jnp.maximum(old[0]-limits.acceleration*dt,jnp.where(sign>0,0.,-limits.speed))
                hi=jnp.minimum(old[0]+limits.acceleration*dt,jnp.where(sign>0,limits.speed,0.))
                lo=jnp.maximum(lo,jnp.where(nonzero,jnp.minimum(a,b),-limits.speed))
                hi=jnp.minimum(hi,jnp.where(nonzero,jnp.maximum(a,b),limits.speed))
                yaw_cap=limits.yaw_rate/jnp.maximum(jnp.abs(curvature),1e-6)
                vx=jnp.clip(requested[0],jnp.maximum(lo,-yaw_cap),jnp.minimum(hi,yaw_cap))
                v=jnp.array([vx,0.]);w=curvature*vx
        elif mode == Mode.PARALLEL:
            # V2 parallel steering is pre-positioned at rest. This segment has
            # one fixed wheel axis; optimizing arbitrary XY at every step would
            # assume instantaneous steering and can flip wheel angles by 180°.
            scalar = jnp.dot(requested[:2],direction)
            old_scalar = jnp.dot(old[:2],direction)
            perpendicular = old[:2]-old_scalar*direction
            delta = jnp.sqrt(jnp.maximum(0.,(limits.acceleration*dt)**2-jnp.sum(perpendicular**2)))
            scalar = jnp.clip(scalar,old_scalar-delta,old_scalar+delta)
            if drive_constraint is not None:
                scalar=jnp.maximum(0.,scalar)
            v = direction*jnp.clip(scalar,-limits.speed,limits.speed)
            w = jnp.array(0.)
        else:
            v = jnp.zeros(2)
            if drive_constraint is not None:
                sign=drive_constraint[1]
                w=jnp.where(sign>0,jnp.maximum(0.,w),jnp.minimum(0.,w))
        result = jnp.array([v[0],v[1],w])
        return result,result
    return jax.lax.scan(step,initial,controls)[1]


def _integrate(controls, pose, dt):
    def step(p,v):
        a = v[2]*dt
        ss = jnp.sinc(a/jnp.pi)*dt
        cc = jnp.where(jnp.abs(a)>1e-5,(1-jnp.cos(a))/jnp.where(jnp.abs(v[2])>1e-9,v[2],1.),.5*v[2]*dt*dt)
        dx,dy = ss*v[0]-cc*v[1],cc*v[0]+ss*v[1]
        c,s = jnp.cos(p[2]),jnp.sin(p[2])
        q = p+jnp.array([c*dx-s*dy,s*dx+c*dy,a])
        return q,q
    return jax.lax.scan(step,pose,controls)[1]


class V2Optimizer(Optimizer):
    def __init__(self, mode, limits=Limits(), body=Body(), samples=256, steps=60, dt=.05, visibility_reserve=.10):
        self.mode,self.limits,self.body = Mode(mode),limits,body
        super().__init__(motion_model='omni', model_dt=dt,time_steps=steps,batch_size=samples,
                         vx_max=limits.speed,vx_min=-limits.speed,vy_max=limits.speed,wz_max=limits.yaw_rate,
                         ax_max=limits.acceleration,ax_min=-limits.acceleration,
                         ay_max=limits.acceleration,ay_min=-limits.acceleration,az_max=limits.yaw_acceleration,
                         vx_std=.20,vy_std=.20,wz_std=.35,temperature=.20,gamma=.001,
                         retry_attempt_limit=0,iteration_count=2,
                         Critics={'PreferForwardCritic':{'enabled':False},
                                  'PathAngleCritic':{'enabled':False},
                                  'TwirlingCritic':{'enabled':False},
                                  'VelocityDeadbandCritic':{'enabled':False},
                                  'PathAlignCritic':{'cost_weight':35.,'threshold_to_consider':.1,'offset_from_furthest':1},
                                  'PathFollowCritic':{'cost_weight':12.,'threshold_to_consider':.1,'offset_from_furthest':2,'furthest_path_lookahead_distance':.5},
                                  'GoalCritic':{'cost_weight':50.,'threshold_to_consider':100.},
                                  'GoalAngleCritic':{'cost_weight':60. if self.mode==Mode.SPIN else 12.,'threshold_to_consider':100.},
                                  'ObstaclesCritic':{'collision_margin_distance':body.clearance+.04+.015,
                                                     'repulsion_distance':.30,'repulsion_weight':1.}},
                         TrajectoryValidator={'collision_lookahead_time':steps*dt,
                                              'collision_margin_distance':body.clearance+.04+.015})
        self.settings_ = replace(self.settings_,shift_control_sequence=False)
        self.critic_manager_ = VisibilityCritics(self.critic_manager_,body,visibility_reserve)

    def updataStateVelocities(self,state):
        controls = jnp.stack((state.cvx,state.cvy,state.cwz),axis=-1)
        controls = controls.at[0].set(jnp.zeros_like(controls[0]))  # explicit braking candidate
        velocities = jax.vmap(lambda u:_project(u,state.speed,self.mode,self.limits,self.settings_.model_dt,state.direction,state.drive_constraint))(controls)
        return replace(state,cvx=velocities[:,:,0],cvy=velocities[:,:,1],cwz=velocities[:,:,2],
                       vx=velocities[:,:,0],vy=velocities[:,:,1],wz=velocities[:,:,2])

    def generateNoisedTrajectories(self,key,state,sequence):
        # Upstream noise/MPPI sampling, plus explicit braking and guidance
        # candidates. A short observed corridor must not depend on a rare
        # independent-noise sample to contain an executable forward rollout.
        k,t = self.settings_.batch_size,self.settings_.time_steps
        key,k1,k2,k3 = jax.random.split(key,4)
        vx = sequence.vx[None,:]+jax.random.normal(k1,(k,t))*self.settings_.sampling_std.vx
        vy = sequence.vy[None,:]+jax.random.normal(k2,(k,t))*self.settings_.sampling_std.vy
        wz = sequence.wz[None,:]+jax.random.normal(k3,(k,t))*self.settings_.sampling_std.wz
        controls = jnp.stack((vx,vy,wz),axis=-1)
        controls = controls.at[0].set(jnp.zeros((t,3)))
        controls = controls.at[1].set(state.reference_controls)
        state = replace(state,cvx=controls[:,:,0],cvy=controls[:,:,1],cwz=controls[:,:,2])
        state = self.updataStateVelocities(state)
        return key,state,self.integrateStateVelocities(state)

    def applyControlSequenceConstraints(self,control_sequence):
        # The inherited update calls this before our measured-state projection.
        return control_sequence

    def updateControlSequence(self,sequence,state,costs):
        sequence,costs = super().updateControlSequence(sequence,state,costs)
        controls = _project(jnp.stack((sequence.vx,sequence.vy,sequence.wz),axis=-1),
                            state.speed,self.mode,self.limits,self.settings_.model_dt,state.direction,state.drive_constraint)
        return ControlSequence(vx=controls[:,0],vy=controls[:,1],wz=controls[:,2]),costs

    def integrateSequenceVelocities(self,sequence,pose):
        return _integrate(jnp.stack((sequence.vx,sequence.vy,sequence.wz),axis=-1),pose,self.settings_.model_dt)

    def integrateStateVelocities(self,state):
        trajectories = jax.vmap(lambda u:_integrate(u,state.pose,self.settings_.model_dt))(
            jnp.stack((state.vx,state.vy,state.wz),axis=-1))
        return Trajectories(x=trajectories[:,:,0],y=trajectories[:,:,1],yaws=trajectories[:,:,2])

    @partial(jax.jit,static_argnames=('self',))
    def evaluate(self,key,sequence,data,pose):
        key,sequence,fail,*_ = self.optimize(key,sequence,data)
        trajectory = self.integrateSequenceVelocities(sequence,pose)
        return key,sequence,trajectory,fail


class ExactController:
    def __init__(self,body=Body(),limits=Limits(),max_points=512,samples=256,steps=60,visibility_reserve=.10):
        if not np.isfinite(visibility_reserve) or not .015 <= visibility_reserve <= .20:
            raise ValueError('visibility reserve must be finite and in [.015, .20]m')
        self.body,self.limits,self.max_points,self.steps = body,limits,max_points,steps
        self.optimizers = {m:V2Optimizer(m,limits,body,samples,steps,visibility_reserve=visibility_reserve) for m in Mode}
        self.footprint = Rectangles(centers=jnp.zeros((1,2)),halfs=jnp.asarray(body.half[None,:]))
        self.identity = None
        self.seeded = set()

    def reset(self,identity=None):
        for optimizer in self.optimizers.values():
            optimizer.reset()
        self.identity = identity
        self.seeded.clear()

    def prewarm(self):
        times = {}
        for m in Mode:
            path = np.zeros((self.steps,3),np.float32)
            path[:,0] = np.linspace(0,.6,self.steps)
            t = time.perf_counter()
            self.compute(np.zeros(3),np.zeros(3),path,m,np.empty((0,2)))
            times[m.name] = time.perf_counter()-t
        self.reset()
        return times

    def compute(self,pose,speed,path,mode,points,visibility=None):
        opt = self.optimizers[Mode(mode)]
        path = np.asarray(path,dtype=np.float32).reshape(-1,3)
        if not len(path) or not np.isfinite(path).all() or not np.isfinite([pose,speed]).all():
            raise ValueError('invalid_optimizer_input')
        if not feasible(speed,mode,self.limits,tolerance=.03):
            raise ValueError('feedback_does_not_match_mode')
        # Guidance may be a two-point exact connection. Nearest-point pruning
        # must see the interior of that line, not choose between distant ends.
        # Keep this contract in the core as well as the ROS adapter.
        path = densify(path,self.body,.025)
        drive_constraint=np.array([0.,1.],np.float32)
        if mode==Mode.SPIN and len(path)>1:
            drive_constraint[1]=1. if np.sum(wrap(np.diff(path[:,2])))>=0 else -1.
        if mode==Mode.DRIVE and len(path)>1:
            ds=np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1)
            dyaw=wrap(np.diff(path[:,2]))
            moving=ds>1e-7
            first=np.flatnonzero(moving)
            if len(first):
                i=first[0];heading=np.array([np.cos(path[i,2]),np.sin(path[i,2])])
                sign=1. if np.dot(path[i+1,:2]-path[i,:2],heading)>=0 else -1.
                curves=dyaw[moving]/(sign*ds[moving])
                if np.ptp(curves)>.002 or np.max(np.abs(curves))>1/self.limits.min_radius+.002:
                    raise ValueError('drive_guidance_changes_curvature')
                curvature=float(np.clip(np.mean(curves),-1/self.limits.min_radius,1/self.limits.min_radius))
                drive_constraint=np.array([curvature,sign],np.float32)
                # If feedback is not on this arc, stop before adopting it.
                # No clipped interval may conceal an acceleration violation.
                lo=max(-self.limits.speed,float(speed[0])-self.limits.acceleration*.05)
                hi=min(self.limits.speed,float(speed[0])+self.limits.acceleration*.05)
                lo=max(lo,0.) if sign>0 else lo
                hi=min(hi,0.) if sign<0 else hi
                if abs(curvature)>1e-6:
                    bounds=sorted(((float(speed[2])-self.limits.yaw_acceleration*.05)/curvature,
                                   (float(speed[2])+self.limits.yaw_acceleration*.05)/curvature))
                    lo,hi=max(lo,bounds[0]),min(hi,bounds[1])
                if lo>hi or (abs(curvature)<=1e-6 and abs(speed[2])>self.limits.yaw_acceleration*.05):
                    raise ValueError('feedback_drive_steering_transition')
        delta = path[-1,:2]-path[0,:2] if len(path)>1 else path[0,:2]-np.asarray(pose)[:2]
        c,s = np.cos(pose[2]),np.sin(pose[2])
        direction = delta@np.array([[c,-s],[s,c]])
        direction = direction/max(float(np.linalg.norm(direction)),1e-9)
        if np.linalg.norm(direction)<.5:
            direction = np.array([1.,0.])
        # The goal critic must not pull through a bend toward a distant segment
        # endpoint. Follow only guidance reachable within this prediction horizon.
        nearest = int(np.argmin(np.linalg.norm(path[:,:2]-np.asarray(pose)[:2],axis=1)
                                +.15*np.abs(wrap(path[:,2]-pose[2]))))
        path = path[nearest:]
        distance = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1))]
        count = max(2,int(np.searchsorted(distance,self.limits.speed*self.steps*.05*.8))+1)
        path = path[:count]
        # Fixed-size path, without changing the geometry of its segments.
        indices = np.linspace(0,len(path)-1,self.steps)
        path = np.column_stack([np.interp(indices,np.arange(len(path)),
                                np.unwrap(path[:,i]) if i==2 else path[:,i]) for i in range(3)])
        points = np.asarray(points,dtype=np.float32).reshape(-1,2)
        if len(points)>self.max_points or not np.isfinite(points).all():
            raise ValueError('unpacked_obstacle_capacity_or_nonfinite')
        packed = np.zeros((self.max_points,2),np.float32)
        mask = np.zeros(self.max_points,np.float32)
        packed[:len(points)],mask[:len(points)] = points,1.
        data = opt.prepare(jnp.asarray(pose,dtype=jnp.float32),jnp.asarray(speed,dtype=jnp.float32),
                           jnp.asarray(path,dtype=jnp.float32),jnp.asarray(path[-1],dtype=jnp.float32),
                           jnp.asarray(packed),jnp.zeros_like(packed),jnp.asarray(mask),
                           self.footprint,jnp.zeros(3))
        cells,observer = visibility if visibility is not None else (np.zeros((80,80)),np.zeros(3))
        if np.asarray(cells).shape!=(80,80) or np.asarray(observer).shape!=(3,):
            raise ValueError('visibility_contract_shape')
        reference = np.zeros((self.steps,3),np.float32)
        if mode in (Mode.DRIVE,Mode.PARALLEL,Mode.SPIN):
            remaining = float(np.dot((path[-1,:2]-np.asarray(pose)[:2])@np.array([[c,-s],[s,c]]),direction)) if mode==Mode.PARALLEL else float(wrap(path[-1,2]-pose[2]))
            limit,acceleration = (self.limits.speed,self.limits.acceleration) if mode==Mode.PARALLEL else (self.limits.yaw_rate,self.limits.yaw_acceleration)
            velocity = float(np.dot(np.asarray(speed)[:2],direction)) if mode==Mode.PARALLEL else float(speed[2])
            if mode==Mode.DRIVE:
                curvature,sign=drive_constraint
                remaining=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum()*sign)
                limit=min(self.limits.speed,self.limits.yaw_rate/max(abs(curvature),1e-6))
                acceleration=min(self.limits.acceleration,self.limits.yaw_acceleration/max(abs(curvature),1e-6))
                velocity=float(speed[0])
            stop_steps = math.ceil(max(self.limits.speed/self.limits.acceleration,self.limits.yaw_rate/self.limits.yaw_acceleration)/.05)
            for i in range(self.steps):
                target = math.copysign(min(limit,.7*math.sqrt(2*acceleration*max(0.,abs(remaining)-.02))),remaining)
                if i>=self.steps-stop_steps:
                    target = 0.
                if mode==Mode.DRIVE:
                    reference[i,0],reference[i,2]=target,drive_constraint[0]*target
                elif mode==Mode.PARALLEL:
                    reference[i,:2] = target*direction
                else:
                    reference[i,2] = target
                velocity = float(np.clip(target,velocity-acceleration*.05,velocity+acceleration*.05))
                remaining -= velocity*.05
        state = V2State(**{field.name:getattr(data.state,field.name) for field in fields(State)},
                        direction=jnp.asarray(direction,dtype=jnp.float32),
                        coverage_cells=jnp.asarray(cells,dtype=jnp.int16),
                        coverage_observer=jnp.asarray(observer,dtype=jnp.float32),
                        coverage_enabled=jnp.asarray(visibility is not None),reference_controls=jnp.asarray(reference),
                        drive_constraint=jnp.asarray(drive_constraint))
        data = replace(data,state=state)
        if Mode(mode) not in self.seeded and mode in (Mode.DRIVE,Mode.PARALLEL,Mode.SPIN):
            # Center the first batch on a model-feasible guidance rollout,
            # rather than averaging predominantly symmetric zero-centered
            # noise after each pause. Braking remains an explicit candidate;
            # optimization and both trajectory validators still run normally.
            nominal=_project(jnp.asarray(reference),jnp.asarray(speed),Mode(mode),
                             self.limits,.05,jnp.asarray(direction),jnp.asarray(drive_constraint))
            opt.control_sequence_=ControlSequence(vx=nominal[:,0],vy=nominal[:,1],wz=nominal[:,2])
            self.seeded.add(Mode(mode))
        opt.key_,sequence,trajectory,fail = opt.evaluate(opt.key_,opt.control_sequence_,data,jnp.asarray(pose,dtype=jnp.float32))
        result,minimum = opt.trajectory_validator_.validateTrajectory(trajectory,jnp.asarray(packed),jnp.asarray(mask),self.footprint)
        controls = np.array(jax.device_get(jnp.stack((sequence.vx,sequence.vy,sequence.wz),axis=-1)))
        trajectory = np.array(jax.device_get(trajectory))
        if bool(fail) or int(result)!=int(ValidationResult.SUCCESS) or not np.isfinite(controls).all():
            opt.reset()
            self.seeded.discard(Mode(mode))
            raise RuntimeError('no_valid_trajectory')
        # Preserve the entire sequence until validation; command is its first
        # sample. Shift only the warm-start, with no post-validation smoothing.
        opt.control_sequence_ = opt.shiftControlSequence(sequence)
        if not all(feasible(v,mode,self.limits) for v in controls):
            raise RuntimeError('mode_constraint_failure')
        differences=np.diff(np.vstack((speed,controls)),axis=0)
        if (np.max(np.linalg.norm(differences[:,:2],axis=1))>self.limits.acceleration*.05+1e-5
                or np.max(np.abs(differences[:,2]))>self.limits.yaw_acceleration*.05+1e-5):
            raise RuntimeError('acceleration_constraint_failure')
        return {'command':controls[0], 'controls':controls,'trajectory':np.vstack((pose,trajectory)),
                'minimum_distance':float(minimum)}
