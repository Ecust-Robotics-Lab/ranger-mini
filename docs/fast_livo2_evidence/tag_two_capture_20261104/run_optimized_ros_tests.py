#!/usr/bin/env python3
"""Run existing C1/C2/Tag ROS suites against isolated build; no devices."""
from pathlib import Path
import importlib,json,sys,time,unittest,hashlib
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'tests'))
import test_fast_livo_localizer_ros as c1
c1.EXECUTABLE=ROOT/'ws_fast_livo/install_c1_confirmation_20261104/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'
results=[]
for name in ['test_fast_livo_localizer_ros','test_fast_livo_relocalizer_ros','test_fast_livo_tags_ros']:
    module=importlib.import_module(name)
    c1.EVIDENCE=HERE/('optimized_'+name)
    c1.EVIDENCE.mkdir(exist_ok=False)
    start=time.monotonic()
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(module))
    results.append({'module':name,'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'skipped':len(result.skipped),'passed':result.wasSuccessful(),'wall_seconds':time.monotonic()-start})
(HERE/'optimized_ros_regressions.json').write_text(json.dumps({'executable':str(c1.EXECUTABLE),'executable_sha256':hashlib.sha256(c1.EXECUTABLE.read_bytes()).hexdigest(),'suites':results},indent=2)+'\n')
raise SystemExit(0 if all(r['passed'] for r in results) else 1)
