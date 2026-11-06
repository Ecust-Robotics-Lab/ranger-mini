"""Read-only capture review; write supplementary evidence outside the original bag."""
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'scripts/fast_livo2'),
               str(ROOT/'ws_nav/src/ranger_localization_bridge')]
from audit_bag import audit, header
from record_session import quality_report, sync_summary
from session_manager import read_session, session_live

FOLDER = ROOT/'bags/fast-livo-relocalization_visit01-trpozzyj'
OUT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parent


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save(name, value):
    with (OUT/name).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


started = time.monotonic()
originals = {str(p.relative_to(FOLDER)): [p.stat().st_size, p.stat().st_mtime_ns, sha(p)]
             for p in FOLDER.rglob('*') if p.is_file() and p.suffix != '.db3'}
db = FOLDER/'raw/raw_0.db3'
before = db.stat()
db_sha = sha(db)
manifest = json.loads((FOLDER/'manifest.json').read_text())
report = audit(FOLDER/'raw')
save('audit.json', report)
quality = quality_report(report, manifest['topics'])
sync = sync_summary(FOLDER/'sensor_assets')
if (not sync['available'] or sync.get('hardware_frame_id_missing') or
        sync.get('hardware_frame_id_nonincreasing') or
        any(status != 'OK' for status in sync.get('status_counts', {}))):
    quality['warnings'].append('camera_sync_anomalies_or_missing_csv')
    if not quality['failures']:
        quality['data_checks'] = 'PARTIAL'
metadata = yaml.safe_load((FOLDER/'raw/metadata.yaml').read_text())['rosbag2_bagfile_information']
expected_counts = {t['topic_metadata']['name']: t['message_count']
                   for t in metadata['topics_with_message_count']}
with sqlite3.connect(db.resolve().as_uri()+'?mode=ro', uri=True) as conn:
    conn.execute('PRAGMA query_only=ON')
    counts = dict(conn.execute('SELECT topics.name,COUNT(messages.id) FROM topics '
                  'LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id'))
    bounds = conn.execute('SELECT MIN(timestamp),MAX(timestamp),COUNT(*) FROM messages').fetchone()
    stamps = {}
    for topic in ('/fast_livo/odometry', '/fast_livo/deskewed_imu'):
        stamps[topic] = [header(row[0])[0] for row in conn.execute(
            'SELECT substr(data,1,256) FROM messages WHERE topic_id='
            '(SELECT id FROM topics WHERE name=?) ORDER BY timestamp,id', (topic,))]
paired = stamps['/fast_livo/odometry'] == stamps['/fast_livo/deskewed_imu']
assert counts == expected_counts and bounds[2] == metadata['message_count']
assert paired and len(set(stamps['/fast_livo/odometry'])) == len(stamps['/fast_livo/odometry'])
sources = manifest['source_files']
assert all(sha(FOLDER/item['file']) == item['sha256'] for item in sources)
config_name = next(item['file'] for item in sources if item['source'].endswith('/ranger_mid360s_hik_trial.yaml'))
cfg = yaml.safe_load((FOLDER/config_name).read_text())['/**']['ros__parameters']
assert cfg['time_offset']['img_time_offset'] == 0.1
assert cfg['common']['require_livo'] and cfg['common']['img_en'] and cfg['imu']['imu_en']
_, state = read_session(ROOT/'.runtime/sessions/fast_livo_record')
assert not session_live(state)
after = db.stat()
assert (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)
assert all([p.stat().st_size, p.stat().st_mtime_ns, sha(p)] == entry
           for name, entry in originals.items() for p in [FOLDER/name])
save('quality.json', quality)
save('integrity.json', {'database': str(db), 'bytes': before.st_size,
     'mtime_ns': before.st_mtime_ns, 'sha256': db_sha, 'source_files_checked': len(sources),
     'original_small_files_checked': len(originals), 'original_files_unchanged': True})
save('capture_summary.json', {'folder': str(FOLDER), 'actual_counts': counts,
     'messages': bounds[2], 'duration_seconds': (bounds[1]-bounds[0])*1e-9,
     'paired_frontend_frames': len(stamps['/fast_livo/odometry']),
     'frontend_stamps_identical': paired, 'sync': sync,
     'gravity_align_en': cfg['uav']['gravity_align_en'], 'img_time_offset': 0.1,
     'session_live': bool(session_live(state)), 'session_state': state['state'],
     'owned': state['owned'], 'capture_complete_marker': (FOLDER/'CAPTURE_COMPLETE').exists(),
     'hardware_started_by_agent': False, 'original_outputs_rewritten': False,
     'data_quality': quality, 'audit_wall_seconds': time.monotonic()-started})
print(json.dumps({'quality': quality, 'messages': bounds[2], 'sha256': db_sha,
                  'paired_frames': len(stamps['/fast_livo/odometry']),
                  'elapsed_seconds': time.monotonic()-started}), flush=True)
