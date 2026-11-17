"""Compare observed action windows; input is finite live-goal evidence JSON."""
import json,sys
from pathlib import Path
import numpy as np
rows=[]
for file in sys.argv[1:]:
 d=json.loads(Path(file).read_text());events=d['events']
 start=next((t for t,e in events if e['state'] not in ('IDLE',)),None)
 end=next((t for t,e in events if e['state'] in ('SUCCEEDED','CANCELED','FAILED')),None)
 if start is None or end is None:continue
 outputs=np.asarray(d['outputs']);outputs=outputs[(outputs[:,0]>=start)&(outputs[:,0]<=end)]
 speeds=np.linalg.norm(outputs[:,1:3],axis=1);nz=speeds[speeds>.001]
 poses=np.asarray(d['poses']);poses=poses[(poses[:,0]>=start)&(poses[:,0]<=end)]
 rows.append({'name':d['name'],'status':d['result_status'],'action_seconds':end-start,
              'zero_fraction':float(np.mean(speeds<=.001)),'mean_command_mps':float(np.mean(speeds)),
              'nonzero_p50_p95_max':np.percentile(nz,[50,95,100]).tolist() if len(nz) else [],
              'net_estimated_translation':float(np.linalg.norm(poses[-1,1:3]-poses[0,1:3])),
              'accepted_plans':sum(e['reason']=='plan_accepted' for _,e in events),
              'final_command':d['outputs'][-1][1:],'final_feedback':d['wheel'][-1][1:]})
print(json.dumps(rows,indent=2))
