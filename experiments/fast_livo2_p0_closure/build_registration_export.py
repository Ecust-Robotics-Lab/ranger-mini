#!/usr/bin/env python3
"""Build only the figure exporter, reusing installed C1/C2 dependency flags."""
import argparse,json,shlex,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
root=Path(__file__).resolve().parents[2];build=root/'ws_fast_livo/online_localization_build/ranger_fast_livo_localizer'
out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
entry=next(x for x in json.loads((build/'compile_commands.json').read_text()) if x['file'].endswith('/test/recovery_replay.cpp'))
compile=shlex.split(entry['command']);obj=out/'registration_figure_export.o'
compile[compile.index('-o')+1]=str(obj);compile[-1]=str(root/'experiments/fast_livo2_p0_closure/registration_figure_export.cpp')
link=shlex.split((build/'CMakeFiles/recovery_replay.dir/link.txt').read_text())
link[link.index('-o')+1]=str(out/'registration_figure_export')
link[next(i for i,s in enumerate(link) if s.endswith('recovery_replay.cpp.o'))]=str(obj)
(out/'build_commands.json').write_text(json.dumps({'compile':compile,'link':link},indent=2)+'\n')
with (out/'build.log').open('w') as log:
 subprocess.run(compile,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
 subprocess.run(link,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
print('PASS isolated exporter build')
