"""Synthetic identical GPU workload, with/without three recorded-data HTTP clients.

This is not the three-fusion real-robot end-to-end latency acceptance.
"""
import http.cookiejar
import json
from pathlib import Path
import threading
import time
import urllib.request
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.exact_controller import ExactController
from ranger_navigation.geometry import Body
from ranger_navigation.models import Mode

def client():
 jar=http.cookiejar.CookieJar();opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),urllib.request.HTTPCookieProcessor(jar))
 req=urllib.request.Request('http://127.0.0.1:8080/api/v1/login',data=json.dumps({'key':(ROOT/'.runtime/navigation_workbench/access.key').read_text().strip()}).encode(),headers={'Origin':'http://127.0.0.1:8080','Content-Type':'application/json'})
 opener.open(req,timeout=3).read();return opener

def main():
 controller=ExactController(Body(),samples=256);controller.prewarm()
 path=np.column_stack((np.linspace(0,.8,60),np.zeros((60,2))))
 a=np.linspace(-np.pi,np.pi,512,endpoint=False);points=np.column_stack((2*np.cos(a),2*np.sin(a)))
 running=threading.Event();errors=[];counts=[];clients=[client() for _ in range(3)];started=threading.Barrier(4)
 def load(index):
  opener=clients[index];count=0
  started.wait()
  while running.is_set():
   start=time.monotonic()
   try:opener.open(f'http://127.0.0.1:8080/api/v1/replays/recorded-u622df4n-v3?time={70+count%40}',timeout=3).read();count+=1
   except Exception as exc:errors.append(str(exc));break
   time.sleep(max(0,.2-(time.monotonic()-start)))
  counts.append(count)
 def measure():
  values=[];controller.reset()
  for _ in range(120):
   start=time.perf_counter();controller.compute(np.zeros(3),np.zeros(3),path,Mode.DRIVE,points);values.append((time.perf_counter()-start)*1000)
  return {f'p{p}':float(np.percentile(values,p)) for p in (50,95,99)}
 baseline=measure();running.set();threads=[threading.Thread(target=load,args=(i,)) for i in range(3)]
 for t in threads:t.start()
 started.wait();time.sleep(.6);loaded=measure();running.clear()
 for t in threads:t.join(5)
 after=measure()
 if sum(counts)<3 or errors:raise RuntimeError('回放负载未实际运行: '+str((counts,errors)))
 result={'workload':'synthetic 256x60 GPU + 512 points; 3 recorded-bag replay clients at 5Hz each','baseline_ms':baseline,'baseline_after_ms':after,'with_clients_ms':loaded,'p95_increment_ms':loaded['p95']-(baseline['p95']+after['p95'])/2,'http_requests':sum(counts),'errors':errors,'real_three_fusion_end_to_end':'PARTIAL'}
 out = ROOT/'.runtime/validation/workbench'
 out.mkdir(parents=True, exist_ok=True)
 (out/'performance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
