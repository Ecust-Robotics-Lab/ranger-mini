#!/usr/bin/env python3
"""Reuse exact frontend compiler/link flags; no package installation or devices."""
import argparse,json,shlex,subprocess
from pathlib import Path
R=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--variant',choices=['baseline','candidate'],required=True);a=p.parse_args()
b=R/f'ws_fast_livo/latency_frontend_{a.variant}_build'
commands=json.loads((R/'ws_fast_livo/latency_frontend_candidate_build/compile_commands.json').read_text())
c=next(x for x in commands if x['file'].endswith('/src/ranger_livo_replay.cpp'))
args=shlex.split(c['command']);args[args.index('-o')+1]=str(b/'latency_sync_checks.o');args[-1]=str(R/'scripts/fast_livo2/latency_sync_checks.cpp')
subprocess.run(args,cwd=c['directory'],check=True)
link=shlex.split((b/'CMakeFiles/ranger_livo_replay.dir/link.txt').read_text())
link[link.index('-o')+1]=str(b/'latency_sync_checks')
i=next(i for i,v in enumerate(link) if v.endswith('/src/ranger_livo_replay.cpp.o'));link[i]=str(b/'latency_sync_checks.o')
subprocess.run(link,cwd=b,check=True)
