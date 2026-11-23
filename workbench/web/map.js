export function localToWorld(x,y,origin){const c=Math.cos(origin[2]),s=Math.sin(origin[2]);return [origin[0]+c*x-s*y,origin[1]+s*x+c*y];}
export function worldToLocal(x,y,origin){const c=Math.cos(origin[2]),s=Math.sin(origin[2]),dx=x-origin[0],dy=y-origin[1];return [c*dx+s*dy,-s*dx+c*dy];}
export class MapView{
 constructor(canvas,onGoal){this.canvas=canvas;this.ctx=canvas.getContext('2d');this.onGoal=onGoal;this.center=[0,0];this.scale=35;this.layers={plan:true,prediction:true,trace:true,presets:true,environment:false};this.presets=[];this.follow=false;this.image=null;this.snap=null;this.goal=null;this.preview=null;this.drag=null;
 new ResizeObserver(()=>{this.resize();this.draw();}).observe(canvas.parentElement);
 canvas.addEventListener('wheel',e=>{e.preventDefault();const a=this.world(e.offsetX,e.offsetY);this.scale=Math.max(5,Math.min(500,this.scale*Math.exp(-e.deltaY*.001)));const b=this.world(e.offsetX,e.offsetY);this.center[0]+=a[0]-b[0];this.center[1]+=a[1]-b[1];this.draw();},{passive:false});
 canvas.addEventListener('contextmenu',e=>e.preventDefault());
 canvas.addEventListener('pointerdown',e=>{canvas.setPointerCapture(e.pointerId);this.drag={screen:[e.offsetX,e.offsetY],center:[...this.center],start:this.world(e.offsetX,e.offsetY),pan:e.shiftKey||e.button===2};if(!this.drag.pan&&this.mode!=='replay'){this.goal=[...this.drag.start,this.goal?.[2]||0];this.draw();}});
 canvas.addEventListener('pointermove',e=>{const p=this.world(e.offsetX,e.offsetY);document.querySelector('#mapCursor').textContent=`map  ${p[0].toFixed(2)}, ${p[1].toFixed(2)} m`;
 if(!this.drag)return;
 if(this.drag.pan){this.follow=false;this.center=[this.drag.center[0]-(e.offsetX-this.drag.screen[0])/this.scale,this.drag.center[1]+(e.offsetY-this.drag.screen[1])/this.scale];}
 else if(this.mode!=='replay'){const a=this.drag.start;this.goal=[...a,Math.atan2(p[1]-a[1],p[0]-a[0])];}this.draw();});
 canvas.addEventListener('pointerup',()=>{if(this.drag&&!this.drag.pan){this.goal??=[...this.drag.start,0];this.onGoal(this.goal);}this.drag=null;});
 canvas.addEventListener('keydown',e=>{if(e.key==='+'||e.key==='=')this.zoom(1.2);if(e.key==='-')this.zoom(1/1.2);});
 }
 resize(){const r=this.canvas.getBoundingClientRect(),d=window.devicePixelRatio||1;this.w=r.width;this.h=r.height;this.canvas.width=Math.round(r.width*d);this.canvas.height=Math.round(r.height*d);this.ctx.setTransform(d,0,0,d,0,0);}
 screen(x,y){return [this.w/2+(x-this.center[0])*this.scale,this.h/2-(y-this.center[1])*this.scale];}
 world(x,y){return [this.center[0]+(x-this.w/2)/this.scale,this.center[1]-(y-this.h/2)/this.scale];}
 zoom(f){this.scale=Math.max(5,Math.min(500,this.scale*f));this.draw();}
 async setMap(map){this.map=map;const image=new Image();image.src=`/api/v1/maps/${map.id}/image`;await image.decode();this.image=image;this.fit();}
 fit(){if(!this.map)return;const m=this.map;this.center=localToWorld(m.width*m.resolution/2,m.height*m.resolution/2,m.origin);this.scale=.88*Math.min(this.w/(m.width*m.resolution),this.h/(m.height*m.resolution));this.draw();}
 setSnapshot(s){this.snap=s;const p=s?.localization?.body_pose;if(this.follow&&p)this.center=p.slice(0,2);this.draw();}
 line(points,color,width=2,dash=[]){if(!points?.length)return;const c=this.ctx;c.beginPath();c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);let first=true;for(const p of points){if(!p){first=true;continue;}const [x,y]=this.screen(...p);if(first)c.moveTo(x,y);else c.lineTo(x,y);first=false;}c.stroke();c.setLineDash([]);}
 marker(p,color,label){const c=this.ctx,[x,y]=this.screen(...p);c.save();c.translate(x,y);c.rotate(-p[2]);c.fillStyle=color;c.beginPath();c.arc(0,0,4,0,Math.PI*2);c.fill();c.strokeStyle=color;c.lineWidth=2;c.beginPath();c.moveTo(0,0);c.lineTo(20,0);c.lineTo(15,-4);c.moveTo(20,0);c.lineTo(15,4);c.stroke();c.restore();if(label){c.font='10px sans-serif';c.fillStyle=color;c.fillText(label,x+8,y-9);}}
 draw(){if(!this.w)return;const c=this.ctx;c.clearRect(0,0,this.w,this.h);c.fillStyle='#e8edf2';c.fillRect(0,0,this.w,this.h);
 if(this.image&&this.map){const m=this.map,[x,y]=this.screen(m.origin[0],m.origin[1]);c.save();c.translate(x,y);c.rotate(-m.origin[2]);c.imageSmoothingEnabled=false;c.globalAlpha=.96;c.drawImage(this.image,0,-m.height*m.resolution*this.scale,m.width*m.resolution*this.scale,m.height*m.resolution*this.scale);c.restore();}
 const extent=[this.world(0,this.h),this.world(this.w,0)],step=this.scale<20?5:1;c.strokeStyle='#8394a018';c.lineWidth=.5;c.beginPath();for(let x=Math.ceil(extent[0][0]/step)*step;x<extent[1][0];x+=step){const px=this.screen(x,0)[0];c.moveTo(px,0);c.lineTo(px,this.h);}for(let y=Math.ceil(extent[0][1]/step)*step;y<extent[1][1];y+=step){const py=this.screen(0,y)[1];c.moveTo(0,py);c.lineTo(this.w,py);}c.stroke();
 const s=this.snap,t=s?.topics||{},env=t['/workbench/environment'];
 if(this.layers.environment&&env?.valid){c.fillStyle='#6bcab533';for(const p of env.coverage||[]){const [x,y]=this.screen(...p);c.fillRect(x-1.5,y-1.5,3,3);}c.fillStyle='#9272b4';for(const p of env.obstacles||[]){const [x,y]=this.screen(...p);c.fillRect(x-2,y-2,4,4);}}
 if(this.layers.presets){const labels=[];for(const p of this.presets){const xy=this.screen(...p.pose),show=!labels.some(v=>Math.hypot(v[0]-xy[0],v[1]-xy[1])<60);if(show)labels.push(xy);this.marker(p.pose,'#8496a7',show?p.name:'');}}
 if(this.layers.plan){const route=t['/navigation/route'];if(route?.segments?.length&&route.history_generation===t['/navigation/state']?.history_generation){for(const segment of route.segments)this.line(segment.poses,['#4b8dde','#9274ba','#e6a549'][segment.mode]||'#4b8dde',2.5);}else this.line(t['/plan']?.points,'#4b8dde',2.5);}
 if(this.preview?.accepted)for(const segment of this.preview.segments||[])this.line(segment.poses,{DRIVE:'#4b8dde',PARALLEL:'#9274ba',SPIN:'#e6a549'}[segment.mode]||'#4b8dde',2,[5,4]);
 if(this.layers.trace)this.line(s?.trace,'#18a98d',2.5);
 const predictionStamp=(t['/navigation/prediction']?.header?.stamp.sec||0)*1e9+(t['/navigation/prediction']?.header?.stamp.nanosec||0),predictionAge=(s?.clock_ns||0)-predictionStamp;
 if(this.layers.prediction&&predictionStamp>0&&predictionAge>=0&&predictionAge<=250e6)this.line(t['/navigation/prediction']?.points,'#e6a549',2.5);
 const loc=s?.localization,p=loc?.body_pose;if(p){const [x,y]=this.screen(...p);c.save();c.translate(x,y);c.rotate(-p[2]);c.fillStyle=loc.valid?'#1eb09b33':'#8596a533';c.strokeStyle=loc.valid?'#169f8a':'#8394a6';c.lineWidth=2;const b=this.map.body;c.fillRect(-b.length/2*this.scale,-b.width/2*this.scale,b.length*this.scale,b.width*this.scale);c.strokeRect(-b.length/2*this.scale,-b.width/2*this.scale,b.length*this.scale,b.width*this.scale);c.restore();this.marker(p,loc.valid?'#079783':'#8595a4','');}
 const target=this.mode==='replay'?s?.task?.pose:this.goal;
 if(target){const [x,y]=this.screen(...target),b=this.map.body;c.save();c.translate(x,y);c.rotate(-target[2]);c.strokeStyle='#de8550';c.fillStyle='#de855022';c.lineWidth=1.5;c.setLineDash([5,3]);c.fillRect(-b.length/2*this.scale,-b.width/2*this.scale,b.length*this.scale,b.width*this.scale);c.strokeRect(-b.length/2*this.scale,-b.width/2*this.scale,b.length*this.scale,b.width*this.scale);c.restore();this.marker(target,'#de8550',this.mode==='replay'?'历史目标':'目标');}
 // Scale bar and map axes remain in physical metres.
 const bar=this.scale<30?2:1,bx=18,by=this.h-55;c.strokeStyle='#6a8192';c.lineWidth=1.5;c.beginPath();c.moveTo(bx,by-4);c.lineTo(bx,by);c.lineTo(bx+bar*this.scale,by);c.lineTo(bx+bar*this.scale,by-4);c.stroke();c.fillStyle='#6a8192';c.font='9px sans-serif';c.fillText(`${bar} m`,bx,by-8);
 }
}
