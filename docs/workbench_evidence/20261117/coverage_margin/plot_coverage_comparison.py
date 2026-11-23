"""Plot the captured failed input; green trajectory is an offline counterfactual."""
from pathlib import Path
import sys,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties
root=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(root/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body
out=Path(__file__).resolve().parent
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']=font.get_name()
with np.load(out/'original_request.npz',allow_pickle=False) as a:
    pose=a['pose'];observer=a['observer_pose'];cells=a['visibility_cells']
with np.load(out/'counterfactual_trajectory.npz',allow_pickle=False) as a:
    trajectory=a['trajectory']
c,s=np.cos(observer[2]),np.sin(observer[2]);r=np.array([[c,-s],[s,c]])
pose=np.r_[(pose[:2]-observer[:2])@r,pose[2]-observer[2]]
trajectory=(trajectory[:,:2]-observer[:2])@r
body=Body();c,s=np.cos(pose[2]),np.sin(pose[2]);r=np.array([[c,-s],[s,c]])
def polygon(margin):
    x,y=body.half+margin
    return np.array([[-x,-y],[x,-y],[x,y],[-x,y],[-x,-y]])@r.T+pose[:2]
fig,axes=plt.subplots(1,2,figsize=(12,5.6),constrained_layout=True)
indexed=np.where(cells<0,0,np.where(cells>=50,2,1))
for ax in axes:
    ax.imshow(indexed,origin='lower',extent=(-2,2,-2,2),cmap=ListedColormap(['#bfc3c7','white','#222222']),vmin=0,vmax=2)
    b=polygon(0);ax.fill(b[:,0],b[:,1],color='#3b9bd5',alpha=.35,label='完整车体 74×52cm')
    ax.plot(b[:,0],b[:,1],color='#2679a4',lw=1)
    ax.set(xlim=(-.55,1.4),ylim=(-.6,.65),xlabel='观测网格 x / m',ylabel='观测网格 y / m',aspect='equal')
    ax.grid(alpha=.15)
    ax.scatter([pose[0]],[pose[1]],s=15,color='#2679a4')
old=polygon(.055);axes[0].plot(old[:,0],old[:,1],color='#c43e36',lw=2,label='旧覆盖外扩 5.5cm')
iy,ix=np.nonzero(cells<0)
unknown=np.column_stack(((ix+.5)*.05-2,(iy+.5)*.05-2))
dx,dy=(unknown-pose[:2]).T;half=body.half+.055;ac,ass=abs(c),abs(s)
intersects=((abs(dx)<=ac*half[0]+ass*half[1]+.025)&
            (abs(dy)<=ass*half[0]+ac*half[1]+.025)&
            (abs(c*dx+s*dy)<=half[0]+.025*(ac+ass))&
            (abs(-s*dx+c*dy)<=half[1]+.025*(ac+ass)))
unknown=unknown[intersects]
assert len(unknown)==4,unknown
axes[0].scatter(unknown[:,0],unknown[:,1],s=120,marker='s',facecolors='none',edgecolors='#c43e36',label='触发拒绝的 4 个未知格')
axes[0].set_title('旧检查：被车体外侧 4 格拦住')
new=polygon(.025);axes[1].plot(new[:,0],new[:,1],color='#16855a',lw=2,label='新覆盖总外扩 2.5cm')
protected=polygon(.015);axes[1].plot(protected[:,0],protected[:,1],color='#b98525',ls=':',lw=1.5,label='扫掠保护范围 1.5cm')
axes[1].plot(trajectory[:,0],trajectory[:,1],color='#16855a',lw=2,label='GPU 候选轨迹（离线）')
axes[1].set_title('新检查：可进入优化，轨迹与制动复核通过')
for ax in axes:ax.legend(loc='upper right',fontsize=8)
fig.suptitle('保存的实车失败输入复核｜灰色未知、白色已观测\n没有向实车发指令；绿线不是实际执行轨迹',fontsize=13)
fig.savefig(out/'coverage_comparison.png',dpi=170)
print(out/'coverage_comparison.png')
