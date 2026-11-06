#!/usr/bin/env python3
"""Read-only final hashes and recording session status."""
from pathlib import Path
import hashlib,json,subprocess,sys
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'tag_two_capture_20261104'

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(4*1024*1024),b''):h.update(data)
    return h.hexdigest()
if sys.argv[1:]!=['--execute']:
    print('DRY-RUN: final integrity checks');raise SystemExit(0)
base=json.loads((BASE/'baseline.json').read_text())
prior=json.loads((BASE/'input_integrity.json').read_text())
result={'captures':[],'maps':[],'formal_files':{},'head_unchanged':False}
for old in prior['captures']:
    cap=ROOT/'bags'/old['capture'];db=next((cap/'raw').glob('*.db3'));s=db.stat()
    got={'sha256':sha(db),'bytes':s.st_size,'mtime_ns':s.st_mtime_ns}
    assert got==old['source']
    manifest=json.loads((cap/'manifest.json').read_text())
    for entry in manifest['source_files']:assert sha(cap/entry['file'])==entry['sha256']
    result['captures'].append({'capture':old['capture'],'unchanged':True,'source':got,'archived_source_files_verified':len(manifest['source_files'])})
for rel,expected in [('outputs/fast-livo-backend-validation-20261028-pcl/snapshot',prior['old_map_manifest_sha256']),('outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot','14658c3b156a05c3f792b9a3a1d90f2c79032f4a379c399715ce338c97faf9ef')]:
    folder=ROOT/rel;digest=sha(folder/'manifest.json');assert digest==expected
    manifest=json.loads((folder/'manifest.json').read_text())
    assert json.loads((folder/'COMMITTED').read_text())['manifest_sha256']==digest
    for filename,digest_file in manifest['files'].items():assert sha(folder/filename)==digest_file
    result['maps'].append({'path':rel,'map_uuid':manifest['map_uuid'],'manifest_sha256':digest,'files_verified':len(manifest['files'])})
for filename,expected in base['captures'][0]['source_shas'].items():
    digest=sha(Path(filename));assert digest==expected;result['formal_files'][filename]=digest
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip();assert head==base['head'];result['head_unchanged']=True
status=json.loads(subprocess.check_output(['bash','scripts/record_fast_livo2.sh','--status'],cwd=ROOT,text=True))
result['record_session']={key:status['session'][key] for key in ['state','owned']};result['record_session']['live']=status['live']
assert not status['live'] and not status['session']['owned']
with (HERE/'final_integrity.json').open('x') as f:f.write(json.dumps(result,indent=2)+'\n')
print(json.dumps({'capture_count':len(result['captures']),'map_file_counts':[m['files_verified'] for m in result['maps']],'formal_files_unchanged':True,'head_unchanged':True,'record_session':result['record_session']},indent=2))
