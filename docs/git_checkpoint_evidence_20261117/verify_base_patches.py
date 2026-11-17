import subprocess,os,json,hashlib,difflib
from pathlib import Path
R=Path('/home/lab307/ranger_nav');B=R/'.runtime/backups/git_checkpoint_20261117';lock=json.loads((R/'config/exact_mppi/sources.lock.json').read_text());out={}
for name in ['ranger_ros2','ugv_sdk']:
 repo=R/'ws_base/src'/name;sha=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip();assert sha==lock['base_sources'][name]
 patch=R/('patches/'+name+'_exact_v2_modes.patch');priors=[R/p for p in lock['base_patch_prerequisites'][name]];files=set()
 for p in [*priors,patch]:
  for line in p.read_text().splitlines():
   if line.startswith('+++ b/'):files.add(line[6:])
 temp=B/(name+'_prerequisites_v2');temp.mkdir();subprocess.run(['git','init','-q',str(temp)],check=True)
 for f in files:
  res=subprocess.run(['git','-C',str(repo),'show','HEAD:'+f],capture_output=True)
  if res.returncode==0:
   dst=temp/f;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(res.stdout)
 for p in priors:
  cmd=['git','-C',str(temp),'apply']
  if subprocess.run([*cmd,'--check',str(p)],capture_output=True).returncode==0:subprocess.run([*cmd,str(p)],check=True)
  elif subprocess.run([*cmd,'--reverse','--check',str(p)],capture_output=True).returncode!=0:raise RuntimeError('prerequisite neither applicable nor already present: '+str(p))
 for f,h in lock['patch_preimages'][name].items():assert hashlib.sha256((temp/f).read_bytes()).hexdigest()==h,(name,f)
 before={f:(temp/f).read_bytes() if (temp/f).exists() else None for f in files}
 subprocess.run(['git','-C',str(temp),'apply',str(patch)],check=True)
 different=[f for f in files if (temp/f).read_bytes()!=(repo/f).read_bytes()]
 if different:
  (B/patch.name).write_bytes(patch.read_bytes());chunks=[]
  for f in sorted(files):
   old=before[f];new=(repo/f).read_bytes()
   chunks.extend(difflib.unified_diff((old.decode() if old else '').splitlines(True),new.decode().splitlines(True),fromfile='a/'+f if old is not None else '/dev/null',tofile='b/'+f))
  patch.write_text(''.join(chunks));lock['patch_sha256'][str(patch.relative_to(R))]=hashlib.sha256(patch.read_bytes()).hexdigest()
  # Verify the refreshed patch independently against the saved prerequisites.
  for f,data in before.items():
   if data is None:(temp/f).unlink(missing_ok=True)
   else:(temp/f).write_bytes(data)
  subprocess.run(['git','-C',str(temp),'apply','--check',str(patch)],check=True);subprocess.run(['git','-C',str(temp),'apply',str(patch)],check=True)
  assert all((temp/f).read_bytes()==(repo/f).read_bytes() for f in files)
 out[name]=dict(head=sha,patch_refreshed=bool(different),different_files_before_refresh=different,postimage_sha256={f:hashlib.sha256((repo/f).read_bytes()).hexdigest() for f in files},reconstruction='PASS')
(R/'config/exact_mppi/sources.lock.json').write_text(json.dumps(lock,indent=2)+'\n');(B/'base_patch_verification.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
