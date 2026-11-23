"""LAN HTTP facade. Explicit allowlist; all task ownership stays in the local daemon."""
import argparse
import asyncio
import hashlib
import json
import os
from pathlib import Path
import secrets
import signal
import sqlite3
import sys
import time
import uuid

from aiohttp import web
from workbench.core import Assets,Presets,Replay,ROOT,atomic,pose

class API:
    def __init__(self,socket,data,map_folders=None):
        self.socket=str(socket);self.data=Path(data);self.data.mkdir(parents=True,exist_ok=True)
        self.assets=Assets(map_folders);self.presets=Presets(self.data/'presets.json',self.assets)
        keyfile=self.data/'access.key'
        if not keyfile.exists():
            fd=os.open(keyfile,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
            with os.fdopen(fd,'w') as f:f.write(secrets.token_urlsafe(32)+'\n')
        self.key=keyfile.read_text().strip();self.sessions=set();self.jobs={};self.processes=set()
        self.snapshot=None;self.snapshot_at=0;self.cache_lock=asyncio.Lock();self.streams=set()
    def authenticated(self,token):
        import hmac
        parts=token.split('.')
        return len(parts)==2 and hmac.compare_digest(parts[1],hmac.new(self.key.encode(),parts[0].encode(),'sha256').hexdigest())
    async def rpc(self,method,data=None):
        reader,writer=await asyncio.wait_for(asyncio.open_unix_connection(self.socket,limit=8_000_000),2)
        try:
            writer.write((json.dumps({'method':method,'data':data or {}},allow_nan=False)+'\n').encode());await writer.drain()
            line=await asyncio.wait_for(reader.readline(),60);reply=json.loads(line)
            if not reply['ok']:raise ValueError(reply['error'])
            return reply['result']
        finally:writer.close();await writer.wait_closed()
    async def snap(self):
        async with self.cache_lock:
            if time.monotonic()-self.snapshot_at>=.09:
                self.snapshot=await self.rpc('snapshot');self.snapshot_at=time.monotonic()
            return self.snapshot
    def catalog(self):
        records=[]
        for folder in sorted((self.data/'records').glob('*')):
            if (folder/'events.sqlite3').exists() and (folder/'result.json').exists():
                try:
                    r=Replay(folder)
                    records.append({'id':folder.name,'name':folder.name,'kind':'light','duration_s':(r.end-r.start)/1e9})
                except (ValueError,OSError,sqlite3.Error):pass
        for folder in sorted((ROOT/'bags').iterdir()):
            if not folder.is_dir() or not ((folder/'localization_result.json').exists() or (folder/'recorder_result.json').exists()):continue
            if not next(folder.glob('**/*.db3'),None):continue
            if not ((folder/'provenance.json').exists() or (folder/'workbench_manifest.json').exists()):continue
            records.append({'id':'bag:'+folder.name,'name':folder.name,'kind':'rosbag'})
        return records
    def replay(self,id):
        if id not in {r['id'] for r in self.catalog() if r['kind']=='light'}:raise ValueError('回放记录不存在或尚未封存')
        return Replay(self.data/'records'/id)
    async def worker(self,job,module,data=None,args=None,ros=False):
        try:
            exe='/usr/bin/python3' if ros else sys.executable
            p=await asyncio.create_subprocess_exec(exe,'-m',module,*(args or []),
                stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.PIPE)
            self.processes.add(p)
            try:out,err=await asyncio.wait_for(p.communicate(json.dumps(data).encode() if data else None),180 if ros else 12)
            except asyncio.TimeoutError:
                p.kill();await p.wait();raise RuntimeError('后台计算超时')
            finally:self.processes.discard(p)
            if p.returncode:raise RuntimeError((err.decode() or out.decode())[-1500:])
            self.jobs[job]={'state':'DONE','result':json.loads(out)}
        except Exception as exc:self.jobs[job]={'state':'FAILED','reason':str(exc)}
    async def route(self,request):
        route=request.match_info.get('route','')
        if request.method=='GET':
            if route=='maps':return web.json_response([self.assets.public(k) for k in self.assets.maps])
            if route.startswith('maps/') and route.endswith('/image'):
                return web.Response(body=self.assets.get(route.split('/')[1])['png'],content_type='image/png',headers={'Cache-Control':'private, max-age=3600'})
            if route=='snapshot':return web.json_response(await self.snap())
            if route=='presets':return web.json_response(self.presets.data)
            if route=='presets/export':return web.json_response(self.presets.export(request.query['map_id']))
            if route=='recordings':return web.json_response(await asyncio.to_thread(self.catalog))
            if route.startswith('jobs/'):return web.json_response(self.jobs.get(route[5:],{'state':'NOT_FOUND'}))
            if route.startswith('replays/'):
                r=await asyncio.to_thread(self.replay,route[8:]);return web.json_response(await asyncio.to_thread(r.seek,float(request.query.get('time',0))))
            if route=='events':
                ws=web.WebSocketResponse(heartbeat=10,max_msg_size=1024);await ws.prepare(request);self.streams.add(ws)
                try:
                    while not ws.closed:
                        started=time.monotonic()
                        try:await asyncio.wait_for(ws.send_json(await self.snap()),1)
                        except (ConnectionError,asyncio.TimeoutError):break
                        await asyncio.sleep(max(.01,.1-(time.monotonic()-started)))
                finally:self.streams.discard(ws);await ws.close()
                return ws
        if request.method=='POST':
            data=await request.json()
            if route=='presets':
                m=self.assets.get(data['map_id'])
                value=self.presets.import_data({'frame':'map','map_uuid':m['map_uuid'],'source_manifest_sha256':m['source_manifest_sha256'],
                    'human_confirmed':True,'presets':[{'name':data['name'],'pose':pose(data['pose'])}]})
                return web.json_response(value)
            if route=='presets/import':return web.json_response(self.presets.import_data(data))
            if route=='previews':
                m=self.assets.get(data['map_id']); data['folder']=m['folder']
                if data.get('profile','standard') not in ('standard','narrow'):raise ValueError('未知净空配置')
                data['goal']=pose(data['goal']);data['start']=pose(data['start'])
                if len(self.jobs)>100:self.jobs={k:v for k,v in self.jobs.items() if v['state']=='RUNNING'}
                if any(v['state']=='RUNNING' for v in self.jobs.values()):raise ValueError('已有后台计算，请稍候')
                job=uuid.uuid4().hex;self.jobs[job]={'state':'RUNNING'}
                asyncio.create_task(self.worker(job,'workbench.preview',data));return web.json_response({'job_id':job},status=202)
            if route=='replays/import':
                if data['id'] not in {r['id'] for r in self.catalog() if r['kind']=='rosbag'}:raise ValueError('未知原包')
                job=uuid.uuid4().hex;folder=ROOT/'bags'/data['id'][4:];out=self.data/'records'/('import-'+job)
                if any(v['state']=='RUNNING' for v in self.jobs.values()):raise ValueError('已有后台计算，请稍候')
                self.jobs[job]={'state':'RUNNING'}
                asyncio.create_task(self.worker(job,'workbench.import_bag',args=['--folder',str(folder),'--output',str(out),*[arg for m in self.assets.maps.values() for arg in ('--map',m['folder'])]],ros=True))
                return web.json_response({'job_id':job},status=202)
            methods={'session/start':'session.start','session/stop':'session.stop','goals':'goal','goals/cancel':'cancel','navigation/reset':'navigation.reset','recordings/start':'raw.start','recordings/stop':'raw.stop'}
            if route in methods:
                result=await self.rpc(methods[route],data)
                self.snapshot_at=0  # A completed click must not display the old cached session.
                return web.json_response(result)
        raise web.HTTPNotFound()

def application(socket,data,map_folders=None):
    api=API(socket,data,map_folders)
    @web.middleware
    async def auth(request,handler):
        try:
            if request.headers.get('Upgrade','').lower()=='websocket' and request.headers.get('Origin')!=f'{request.scheme}://{request.host}':raise web.HTTPForbidden(text='来源不匹配')
            if request.method not in ('GET','HEAD'):
                if request.headers.get('Origin')!=f'{request.scheme}://{request.host}':raise web.HTTPForbidden(text='来源不匹配')
                if request.content_type!='application/json':raise web.HTTPUnsupportedMediaType()
            if request.path=='/api/v1/login' and request.method=='POST':
                body=await request.json()
                if not secrets.compare_digest(str(body.get('key','')),api.key):
                    await asyncio.sleep(.3);raise web.HTTPUnauthorized(text='访问密钥错误')
                token=secrets.token_hex(16);token=token+'.'+__import__('hmac').new(api.key.encode(),token.encode(),'sha256').hexdigest()
                response=web.json_response({'ok':True});response.set_cookie('ranger_session',token,httponly=True,samesite='Strict',max_age=43200);return response
            if request.path.startswith('/api/') and not api.authenticated(request.cookies.get('ranger_session','')):
                raise web.HTTPUnauthorized(text='请登录')
            response=await handler(request)
            response.headers['X-Content-Type-Options']='nosniff'
            response.headers['Content-Security-Policy']="default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'"
            response.headers.setdefault('Cache-Control','no-store')
            return response
        except (ValueError,KeyError,TypeError) as exc:return web.json_response({'error':str(exc)},status=400)
        except (ConnectionError,OSError,asyncio.TimeoutError) as exc:return web.json_response({'error':'任务服务不可用: '+str(exc)},status=503)
    app=web.Application(middlewares=[auth],client_max_size=256_000);app['api']=api
    app.router.add_route('*','/api/v1/{route:.*}',api.route)
    async def index(_):return web.FileResponse(ROOT/'workbench/web/index.html')
    app.router.add_get('/',index);app.router.add_static('/assets',ROOT/'workbench/web')
    async def shutdown(_):
        for ws in list(api.streams):await ws.close(code=1001)
        for p in list(api.processes):
            if p.returncode is None:p.terminate()
    app.on_shutdown.append(shutdown)
    return app

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--socket',required=True);p.add_argument('--data',required=True);p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8080);p.add_argument('--map',dest='maps',action='append');a=p.parse_args()
    web.run_app(application(a.socket,a.data,a.maps),host=a.host,port=a.port,print=lambda x:print(x,flush=True))
