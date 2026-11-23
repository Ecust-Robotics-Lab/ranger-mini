from pathlib import Path
import json,sys,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.patches import Polygon
root=Path.cwd();sys.path.insert(0,str(root/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body
out=root/'docs/workbench_evidence/20261117/goal_freshness'
if Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').exists():
 from matplotlib.font_manager import FontProperties
 plt.rcParams['font.family']=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
with np.load(out/'original_request.npz',allow_pickle=False) as a:
 meta=json.loads(str(a['meta']));cells=a['cells'];start=a['pose'];goal=a['goal'];points=a['points']
r=meta['resolution'];origin=meta['origin'];extent=[origin[0],origin[0]+cells.shape[1]*r,origin[1],origin[1]+cells.shape[0]*r]
fig,axes=plt.subplots(1,2,figsize=(12,6),constrained_layout=True)
def rect(ax,p,margin,color,label):
 h=Body().half+margin;c,s=np.cos(p[2]),np.sin(p[2]);v=np.array([[h[0],h[1]],[h[0],-h[1]],[-h[0],-h[1]],[-h[0],h[1]]])@np.array([[c,s],[-s,c]])+p[:2]
 ax.add_patch(Polygon(v,fill=False,edgecolor=color,lw=2,label=label))
for ax,p,title in zip(axes,[start,goal],['起点：真实车体可放下，旧静态裕量过大','原目标：实时障碍点已进入车体范围']):
 ax.imshow(cells,origin='lower',extent=extent,cmap=ListedColormap(['#ccc','#fff','#222']),norm=BoundaryNorm([-1.5,-.5,49.5,100.5],3),interpolation='nearest')
 ax.scatter(points[:,0],points[:,1],s=4,c='#c69154',alpha=.6,label='已记录的实时障碍')
 rect(ax,p,0,'#1998b2','真实车体 0.74×0.52m');ax.plot(*p[:2],'o',color='#1998b2')
 ax.set_xlim(p[0]-.85,p[0]+.85);ax.set_ylim(p[1]-.85,p[1]+.85);ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('map x / m');ax.set_ylabel('map y / m')
rect(axes[0],start,.095,'#cc4c4c','旧标准静态总裕量 9.5cm');rect(axes[0],start,.055,'#14924e','新标准静态总裕量 5.5cm')
inside=points[Body().distances(goal,points)<0];axes[1].scatter(inside[:,0],inside[:,1],s=35,marker='x',c='#d33',label='进入目标车体的障碍点')
for ax in axes:ax.legend(loc='lower right',fontsize=8)
fig.suptitle('原失败请求的离线复核；定位为记录估计，不是当前车位或实车通行成绩')
fig.savefig(out/'endpoint_review.png',dpi=170)
