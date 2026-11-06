#!/usr/bin/env bash
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: build pinned official STD/Scan Context offline benchmark; no devices or system installation.'
  exit 0
fi
cd "$root"
python3 - <<'PY'
import json
from pathlib import Path
import subprocess
root=Path('outputs/fast-livo-demo-retrieval-comparison-20261106/deps')
root.mkdir(parents=True,exist_ok=True)
lock=json.loads(Path('config/fast_livo2/retrieval_benchmark_sources.lock.json').read_text())
for repo in lock['repositories']:
 p=root/('STD' if repo['name']=='STD' else 'scancontext_tro')
 if not p.exists():
  subprocess.run(['git','clone',repo['origin'],str(p)],check=True)
  subprocess.run(['git','-C',str(p),'checkout','--detach',repo['commit']],check=True)
 actual=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
 if actual!=repo['commit']: raise RuntimeError('Existing source SHA mismatch: '+str(p))
 if subprocess.check_output(['git','-C',str(p),'status','--porcelain']): raise RuntimeError('Existing source modified: '+str(p))
PY
python3 scripts/fast_livo2/retrieval_benchmark/prepare_upstream.py outputs/fast-livo-demo-retrieval-comparison-20261106
source /opt/ros/humble/setup.bash
source ws_fast_livo/online_install/setup.bash
export CMAKE_PREFIX_PATH="$root/ws_fast_livo/relocalization_deps:$root/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
cmake -S scripts/fast_livo2/retrieval_benchmark -B outputs/fast-livo-demo-retrieval-comparison-20261106/build \
  -DCMAKE_BUILD_TYPE=Release -DADAPTED="$root/outputs/fast-livo-demo-retrieval-comparison-20261106/adapted"
cmake --build outputs/fast-livo-demo-retrieval-comparison-20261106/build -j2
