from pathlib import Path
import os,json,subprocess,sys
root=Path.cwd();sys.path[:0]=[str(root/'scripts/fast_livo2'),str(root/'scripts'),str(root/'ws_nav/src/ranger_localization_bridge')]
import mapping_session as mapping
from fast_livo2.frontend_upgrade import declaration,sha
from ranger_localization_bridge.runtime_config import resolve
out=root/'docs/workbench_evidence/20261117/goal_freshness';os.environ['RANGER_RUNTIME_CONFIG']=str(root/'config/profiles/fast_livo2_frontend.yaml');cfg=resolve(root)
current=mapping.frontend_identity(cfg);legacy=mapping.frontend_identity(cfg,legacy=True);archive=root/'outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map';before=sha(archive/'manifest.json');decl=declaration(archive,current,legacy)
identity=out/'current_frontend_identity.json';identity.write_text(json.dumps(current,indent=2)+'\n')
command=[str(root/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/archive_tool'),'--archive',str(archive),'--identity-file',str(identity)]
rejected=subprocess.run(command,capture_output=True,text=True);print('Without explicit upgrade expected reject:',rejected.returncode,rejected.stderr)
assert rejected.returncode!=0
passed=subprocess.run(command+['--frontend-upgrade',str(decl)],capture_output=True,text=True);print(passed.stdout,passed.stderr);assert passed.returncode==0
assert before==sha(archive/'manifest.json')
(out/'real_map_upgrade.json').write_text(json.dumps({'pass':True,'without_upgrade_exit':rejected.returncode,'with_upgrade_exit':passed.returncode,'upgrade_path':str(decl),'old_map_manifest_unchanged':True,'source_identity_changed':current['frontend_sources_sha256']!=legacy['frontend_sources_sha256'],'sensor_and_time_unchanged':all(current[k]==legacy[k] for k in ['sensor_configuration_sha256','time_contract_sha256'])},indent=2)+'\n')
