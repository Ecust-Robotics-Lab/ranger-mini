#!/usr/bin/env python3
"""Reproduce audited project-local dependencies from locked debs and bundled Pangolin."""
import argparse,hashlib,json,subprocess,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');a=ap.parse_args()
    records=json.loads((ROOT/'docs/04_evidence/dependencies.json').read_text())
    entries=[]
    for rec in records:
        if 'file' not in rec:continue
        meta=dict(x.split(': ',1) for x in rec['metadata'].splitlines())
        entries.append((meta['Package']+'='+meta['Version'],Path(rec['file']).name,rec['sha256']))
    unwind=json.loads((ROOT/'docs/04_evidence/unwind_runtime.json').read_text())
    entries.append(('libunwind8=1.3.2-2build2.1','libunwind8_1.3.2-2build2.1_amd64.deb',unwind['sha256']))
    print(json.dumps({'dry_run':not a.execute,'packages':[x[0] for x in entries],'parallel':2,'prefix':str(ROOT/'ws_slam/deps')},indent=2))
    if not a.execute:return
    d=ROOT/'ws_slam/deps';cache=d/'downloads';cache.mkdir(parents=True,exist_ok=True)
    for pkg,name,digest in entries:
        p=cache/name
        if not p.exists():subprocess.run(['apt-get','download',pkg],cwd=cache,check=True)
        if sha(p)!=digest:raise RuntimeError('Checksum mismatch: '+name)
    # Only unpack into a new prefix; do not overwrite a previously prepared tree.
    if not (d/'root').exists():
        (d/'root').mkdir()
        for _,name,_ in entries:subprocess.run(['dpkg-deb','-x',str(cache/name),str(d/'root')],check=True)
    t=d/'root/usr/lib/x86_64-linux-gnu/cmake/glog/glog-targets.cmake'
    original='/usr/lib/x86_64-linux-gnu/libunwind.so';relocated='${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libunwind.so'
    s=t.read_text()
    if original in s:t.write_text(s.replace(original,relocated))
    elif relocated not in s:raise RuntimeError('Unexpected glog export; inspect before editing')
    archive=ROOT/'ws_slam/src/lightning/thirdparty/Pangolin-0.9.3.zip'
    if sha(archive)!='bd22e2f7cbf7093fcc8e20927157dccd5a82057ab187c1450af37a112bce5807':raise RuntimeError('Pangolin checksum mismatch')
    dest=d/'src/Pangolin-0.9.3'
    if not dest.exists():
        (d/'src').mkdir(exist_ok=True)
        with zipfile.ZipFile(archive) as z:
            for name in z.namelist():
                if not (d/'src'/name).resolve().is_relative_to((d/'src').resolve()):raise RuntimeError('Unsafe zip path')
            z.extractall(d/'src')
    for rec in json.loads((ROOT/'docs/04_evidence/pangolin_commands.json').read_text()):
        subprocess.run(rec['command'],cwd=ROOT,check=True)
if __name__=='__main__':main()
