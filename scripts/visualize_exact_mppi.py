#!/usr/bin/env python3
"""Local evidence viewer: recorded stationary queries and separate simulation.

Use the project JAX environment for --stage recorded, system matplotlib for
--stage render. Starts no ROS graph/devices and never changes original evidence.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT/'docs/exact_mppi_evidence/20261110'
NAMES = {'forward':'前进','reverse':'倒车','lateral':'横移','diagonal':'斜行',
         'spin':'原地旋转','left_turn':'左转','right_turn':'右转',
         'static_obstacle':'静态绕障','door':'窄门通过'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rounded(value):
    return np.asarray(value).round(4).tolist()


def export_recorded(folder):
    sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
    from ranger_navigation.geometry import load_geometry,validate_sweep
    from ranger_navigation.obstacles import ObstacleMemory
    from ranger_navigation.exact_controller import ExactController
    from ranger_navigation.models import Mode
    from exact_mppi.mppi_jax.logger import logger
    logger.setLevel(40)
    inputs = ROOT/'.runtime/exact_recorded_inputs.npz'
    provenance = json.loads((EVIDENCE/'recorded_height_rays/extraction.json').read_text())
    if sha(inputs)!=provenance['inputs_sha256']:
        raise RuntimeError('recorded input identity changed; do not substitute a different capture')
    body,_,_ = load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml')
    controller = ExactController(body)
    controller.prewarm()
    rows = provenance['selected_queries']
    frames = []
    with np.load(inputs,allow_pickle=False) as data:
        for i,row in enumerate(rows):
            pose,mode = data['pose_'+str(i)],Mode(i%3)
            path = np.tile(pose,(60,1))
            if mode==Mode.SPIN:
                path[:,2] += np.linspace(0,.2,60)
            else:
                path[:,:2] += np.linspace(0,.3,60)[:,None]*[np.cos(pose[2]),np.sin(pose[2])]
            points,full = data['points_'+str(i)],data['full_'+str(i)]
            cells,observer = data['visibility_'+str(i)],data['sensor_'+str(i)]
            memory = ObstacleMemory(body)
            memory.set_visibility(observer,data['ranges_'+str(i)],cells)
            prediction,command,minimum,valid,reason = [],[0.,0.,0.],None,False,'no_valid_trajectory'
            start = time.perf_counter()
            try:
                result = controller.compute(pose,np.zeros(3),path,mode,points,(cells,observer))
                valid,minimum,reason = validate_sweep(result['trajectory'],full,body,
                    coverage=memory.covers,margin=body.clearance+.04)
                if valid:
                    prediction,command = rounded(result['trajectory']),rounded(result['command'])
            except RuntimeError as error:
                reason = str(error)
            frame = {'time':(row['stamp_ns']-rows[0]['stamp_ns'])/1e9,
                'stamp_ns':str(row['stamp_ns']),'pose':rounded(pose),'mode':mode.name,
                'valid':bool(valid),'reason':reason,'command':command,
                'prediction':prediction,'guidance':rounded(path),'points':rounded(full),
                'sampled':rounded(points),'observer':rounded(observer),
                'known':base64.b64encode(np.packbits(cells.ravel()>=0,bitorder='little')).decode(),
                'minimum':float(minimum) if minimum is not None and np.isfinite(minimum) else None,
                'visualization_compute_ms':round((time.perf_counter()-start)*1000,3)}
            frames.append(frame)
    report = {'kind':'recorded_inputs_with_new_stationary_optimizer_queries',
        'source_bag':provenance['source_bag'],'db_sha256':provenance['db_sha256'],
        'inputs_sha256':provenance['inputs_sha256'],'body':[body.length,body.width],
        'hardware_started':False,'commands_published':False,'initial_velocity':[0.,0.,0.],
        'frames':frames,'accepted':sum(f['valid'] for f in frames),
        'rejected':sum(not f['valid'] for f in frames),
        'notice':'Real recorded clouds and interpolated wheel poses; predictions newly recomputed for viewing, never executed. No closed-loop navigation claim.'}
    target = folder/'recorded_queries.json'
    if target.exists():
        raise FileExistsError(target)
    target.write_text(json.dumps(report,separators=(',',':'))+'\n')
    print(json.dumps({k:report[k] for k in ['kind','accepted','rejected','commands_published']}),flush=True)


def configure_plot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,
        'axes.spines.top':False,'axes.spines.right':False,'font.size':10,
        'figure.facecolor':'white','axes.facecolor':'#f5f7fb','grid.color':'#dde3ed'})
    return plt


def body_patch(ax,pose,color='#0891b2',alpha=.18,arrow=True):
    from matplotlib.patches import Polygon
    p = np.asarray(pose)
    c,s = np.cos(p[2]),np.sin(p[2])
    rotation = np.array([[c,s],[-s,c]])
    corners = np.array([[.37,.26],[.37,-.26],[-.37,-.26],[-.37,.26]])@rotation+p[:2]
    patch = Polygon(corners,closed=True,facecolor=color,edgecolor=color,alpha=alpha,lw=1.8)
    ax.add_patch(patch)
    if arrow:
        ax.annotate('',xy=p[:2]+np.array([c,s])*.36,xytext=p[:2],
                    arrowprops={'arrowstyle':'->','color':color,'lw':2})
    return patch


def plot_recorded(ax,frame,title):
    import matplotlib.colors as colors
    p,o = np.asarray(frame['pose']),np.asarray(frame['observer'])
    known = np.unpackbits(np.frombuffer(base64.b64decode(frame['known']),np.uint8),bitorder='little').reshape(80,80)
    ax.set_facecolor('#dfe5ee')
    ax.imshow(known,origin='lower',extent=[o[0]-p[0]-2,o[0]-p[0]+2,o[1]-p[1]-2,o[1]-p[1]+2],
              cmap=colors.ListedColormap(['#dfe5ee','#ffffff']),vmin=0,vmax=1,interpolation='nearest')
    points = np.asarray(frame['points'])-p[:2]
    ax.scatter(points[:,0],points[:,1],s=3,color='#d97706',label='实录障碍体素')
    guide = np.asarray(frame['guidance'])-p
    ax.plot(guide[:,0],guide[:,1],'--',color='#2563eb',lw=1.5,label='短程查询参考（非全局路径）')
    if frame['prediction']:
        prediction = np.asarray(frame['prediction'])-np.r_[p[:2],0.]
        ax.plot(prediction[:,0],prediction[:,1],color='#7c3aed',lw=2.5,label='未执行的局部预测')
        body_patch(ax,prediction[-1],color='#7c3aed',alpha=.12,arrow=False)
    body_patch(ax,[0.,0.,p[2]],alpha=.23)
    ax.set(xlim=(-2.15,2.15),ylim=(-2.15,2.15),aspect='equal',
           xlabel='odom X 相对本帧基座 / m',ylabel='odom Y 相对本帧基座 / m',title=title)
    ax.legend(loc='upper left',fontsize=8,framealpha=.9)


def plot_simulation(ax,scenario):
    poses = np.array([f['pose'] for f in scenario['frames']])
    points = np.array(scenario['obstacles']).reshape(-1,2)
    if len(points):
        ax.scatter(points[:,0],points[:,1],s=14,marker='s',color='#ef4444',label='仿真障碍')
    for i,seg in enumerate(scenario['route_history'][0]['segments']):
        route = np.array(seg['poses'])
        ax.plot(route[:,0],route[:,1],'--',color='#2563eb',lw=1.2,label='初始全局路径' if i==0 else None)
    ax.plot(poses[:,0],poses[:,1],color='#059669',lw=2.3,label='仿真实际走过的轨迹')
    body_patch(ax,poses[0],color='#94a3b8',alpha=.1)
    for i in np.linspace(0,len(poses)-1,6).astype(int):
        body_patch(ax,poses[i],alpha=.14,arrow=False)
    goal = scenario['goal']
    ax.plot(goal[0],goal[1],marker='*',ms=13,color='#7c3aed',label='目标')
    xy = np.vstack((poses[:,:2],np.array(goal[:2])[None,:]))
    lo,hi = xy.min(axis=0)-.65,xy.max(axis=0)+.65
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='仿真 X / m',ylabel='仿真 Y / m',
           title=f"闭环仿真 · {NAMES[scenario['name']]} · {scenario['result']['state']}")
    ax.grid(alpha=.6)
    ax.legend(loc='upper left',fontsize=8,framealpha=.9)


def simulation_animation(folder,scenario,plt):
    """Export the same model trace as a GIF, with no invented interpolation."""
    from PIL import Image
    fig,ax = plt.subplots(figsize=(7.2,5.2),dpi=100,constrained_layout=True)
    poses = np.array([f['pose'] for f in scenario['frames']])
    points = np.asarray(scenario['obstacles']).reshape(-1,2)
    if len(points):
        ax.scatter(points[:,0],points[:,1],s=20,marker='s',color='#ef4444',label='仿真障碍')
    for i,seg in enumerate(scenario['route_history'][0]['segments']):
        route = np.array(seg['poses'])
        ax.plot(route[:,0],route[:,1],'--',color='#2563eb',lw=1,label='初始路径' if i==0 else None)
    line, = ax.plot([],[],color='#059669',lw=2.5,label='模型已走轨迹')
    goal = scenario['goal']
    ax.plot(goal[0],goal[1],marker='*',ms=14,color='#7c3aed',label='目标')
    body = body_patch(ax,poses[0],alpha=.25,arrow=False)
    heading, = ax.plot([],[],color='#087e93',lw=2.5)
    lo,hi = poses[:,:2].min(axis=0)-.65,poses[:,:2].max(axis=0)+.65
    ax.set(xlim=(lo[0],max(hi[0],goal[0]+.4)),ylim=(lo[1],hi[1]),aspect='equal',
           xlabel='仿真 X / m',ylabel='仿真 Y / m')
    ax.legend(loc='upper left',fontsize=9)
    ax.grid(alpha=.6)
    images=[]
    for i in np.linspace(0,len(poses)-1,120).round().astype(int):
        p=poses[i];c,s=np.cos(p[2]),np.sin(p[2])
        body.set_xy(np.array([[.37,.26],[.37,-.26],[-.37,-.26],[-.37,.26]])@np.array([[c,s],[-s,c]])+p[:2])
        heading.set_data([p[0],p[0]+.32*c],[p[1],p[1]+.32*s])
        line.set_data(poses[:i+1,0],poses[:i+1,1])
        f=scenario['frames'][i]
        ax.set_title(f"闭环仿真 · 静态绕障（没有实车运动）\n{f['time']:.1f} / {scenario['frames'][-1]['time']:.1f} 模拟秒 · {f['mode']} · {f['state']}")
        fig.canvas.draw()
        image=Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy())
        images.append(image.convert('P',palette=Image.ADAPTIVE,colors=128))
    images[0].save(folder/'simulation_obstacle.gif',save_all=True,append_images=images[1:],
                   duration=100,loop=0,optimize=True)
    plt.close(fig)


def render(folder):
    recorded = json.loads((folder/'recorded_queries.json').read_text())
    sim = json.loads((folder/'simulation_trace.json').read_text())
    assert len(sim['scenarios'])==9 and all(s['result']['pass'] for s in sim['scenarios'])
    frames = recorded['frames']
    # Show a meaningful translating prediction and a genuine rejected frame.
    accepted = max((f for f in frames if f['valid']),key=lambda f:np.linalg.norm(np.array(f['prediction'])[-1,:2]-f['pose'][:2]))
    rejected = min((f for f in frames if not f['valid']),key=lambda f:np.linalg.norm(np.asarray(f['pose'])[:2]-accepted['pose'][:2]))
    selected = {s['name']:s for s in sim['scenarios']}
    plt = configure_plot()
    fig,axes = plt.subplots(2,2,figsize=(13,11),constrained_layout=True)
    plot_recorded(axes[0,0],accepted,f"实录环境 + 新增静止查询 · 通过 · {accepted['mode']}")
    plot_recorded(axes[0,1],rejected,f"实录环境 + 新增静止查询 · 停车拒绝 · {rejected['mode']}")
    plot_simulation(axes[1,0],selected['static_obstacle'])
    plot_simulation(axes[1,1],selected['door'])
    fig.suptitle('EXACT-MPPI 离线证据可视化\n上：录包数据与未执行预测　｜　下：合成场景闭环仿真',fontsize=17)
    fig.savefig(folder/'overview.png',dpi=145)
    plt.close(fig)
    simulation_animation(folder,selected['static_obstacle'],plt)
    payload = {'recorded':recorded,'simulation':sim,'names':NAMES,
        'recorded_default':frames.index(accepted),
        'historical_benchmark':json.loads((EVIDENCE/'recorded_contention_height_rays/report.json').read_text())}
    template = (ROOT/'scripts/exact_mppi_visualization.html').read_text()
    html = template.replace('__EVIDENCE_PAYLOAD__',json.dumps(payload,separators=(',',':')).replace('</','<\\/'))
    (folder/'index.html').write_text(html)
    manifest = {'hardware_started':False,'commands_published':False,
        'recorded_query_rerun':{'accepted':recorded['accepted'],'rejected':recorded['rejected']},
        'simulation_passed':len(sim['scenarios']),'source_bag':recorded['source_bag'],
        'recorded_inputs_sha256':recorded['inputs_sha256'],
        'sources':{str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),ROOT/'scripts/validate_exact_mppi.py',ROOT/'scripts/exact_mppi_visualization.html']},
        'files':{p.name:sha(p) for p in folder.iterdir() if p.is_file() and p.name!='visualization_manifest.json' and p.suffix in ('.json','.html','.png','.gif')},
        'notice':'Recorded predictions are new stationary queries. Simulation is separate. No physical navigation performed.'}
    (folder/'visualization_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'html':str(folder/'index.html'),'preview':str(folder/'overview.png'),
                      'recorded_rerun':manifest['recorded_query_rerun'],'simulation_passed':len(sim['scenarios'])}),flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=['recorded','render'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    if args.stage=='recorded':export_recorded(args.output)
    else:render(args.output)


if __name__=='__main__':main()
