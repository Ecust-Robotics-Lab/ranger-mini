#!/usr/bin/env python3
"""Derive ROS-free benchmark cores; preserve upstream repositories and record diff."""
from pathlib import Path
import difflib
import json
import shutil
import subprocess
import sys

root = Path(sys.argv[1]).resolve()
out = root/'adapted'; out.mkdir(exist_ok=True)
manifest = {}


def remove_function(s, signature):
    start = s.index(signature); opening = s.index('{', start); depth = 1; i = opening+1
    while depth:
        depth += (s[i]=='{')-(s[i]=='}'); i += 1
    return s[:start]+s[i:]


std = root/'deps/STD'; sc = root/'deps/scancontext_tro'
for name, source in [('STD',std), ('ScanContext',sc)]:
    manifest[name] = {'sha': subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'], text=True).strip(),
                      'status': subprocess.check_output(['git','-C',str(source),'status','--short'], text=True),
                      'origin': subprocess.check_output(['git','-C',str(source),'remote','get-url','origin'], text=True).strip()}
for source in [std/'include/STDesc.h', std/'src/STDesc.cpp']:
    s = original = source.read_text()
    if source.suffix=='.h':
        s = '\n'.join(line for line in s.splitlines() if not any(v in line for v in ('#include <ros/', '#include <visualization_msgs/')))+'\n'
        s = s.replace('void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting);', '')
        start=s.index('void publish_std_pairs('); end=s.index(';',start)+1; s=s[:start]+s[end:]
        s = s.replace('private:', 'public: // Exposed only for fixed-map top-K benchmarking.')
        s = '#include <pcl/point_types.h>\n#include <pcl/point_cloud.h>\n#include <chrono>\n'+s
    else:
        s = s.replace('#include "include/STDesc.h"','#include "STDesc.h"')
        s = remove_function(s, 'void read_parameters(')
        s = remove_function(s, 'void publish_std_pairs(')
        s = remove_function(s, 'void STDescManager::PlaneGeomrtricIcp(')
        s = s.replace('ROS_ERROR_STREAM(', 'PCL_ERROR_STREAM(')
    (out/source.name).write_text(s)
    (out/(source.name+'.patch')).write_text(''.join(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile=str(source),tofile=source.name)))
scdir = sc/'cpp/module/Scancontext'
for source in scdir.iterdir():
    if source.is_file(): shutil.copy2(source,out/source.name)
p = out/'Scancontext.h'; original = s = p.read_text()
s = '\n'.join(line for line in s.splitlines() if not any(v in line for v in ('#include <cv_bridge/', '#include <pcl_conversions/')))+'\n'
# 20 m is frozen for this indoor benchmark; 80 m stock configuration is also tested.
s = s.replace('const double PC_MAX_RADIUS = 80.0;', 'double PC_MAX_RADIUS = 80.0;')
s = s.replace('const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);', 'double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);')
p.write_text(s)
(out/'Scancontext.h.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile=str(scdir/'Scancontext.h'),tofile='Scancontext.h')))
(out/'upstream.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest))
