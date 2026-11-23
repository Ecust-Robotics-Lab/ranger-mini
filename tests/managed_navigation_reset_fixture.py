"""Test-only guardian: actual navigation node, compute health only, isolated DDS."""
import json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from session_manager import Component,run_guarded
if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='227':
 raise RuntimeError('Isolated domain227 only')
directory=Path(sys.argv[1]);command=json.loads(Path(sys.argv[2]).read_text())
raise SystemExit(run_guarded(directory,[Component('navigation',command,ready_key='navigation',resource='isolated_navigation_reset_check')],
    startup_timeout=40.,fail_on_health_loss=True,health_loss_grace=1.,cleanup_grace=5.))
