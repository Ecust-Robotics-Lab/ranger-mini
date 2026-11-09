"""Read original field04 bag/CSV only; print observed failure diagnostics as JSON."""
import collections
import csv
import json
from pathlib import Path
import sqlite3
import statistics

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root = Path(__file__).resolve().parents[3]
capture = root / 'bags/fast-livo-localization-88k5eclm'
db = sqlite3.connect(f'file:{capture / "raw/raw_0.db3"}?mode=ro', uri=True)
topic, kind = db.execute(
    "SELECT id,type FROM topics WHERE name='/fast_livo/localizer/estimate'"
).fetchone()
message_type = get_message(kind)
rows = []
for timestamp, data in db.execute(
    'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (topic,)
):
    message = deserialize_message(data, message_type)
    rows.append((timestamp, message, json.loads(message.quality_json or '{}')))
db.close()
start = next(i for i, (_, m, _) in enumerate(rows) if m.state == 3)
end = next(i for i in range(start, len(rows)) if rows[i][1].pose_valid)
t0 = rows[start][0]
observed = []
previous = None
for timestamp, message, quality in rows[start:end + 1]:
    key = (message.reason, quality.get('query_stamp_ns'), quality.get('elapsed_ms'))
    if key == previous:
        continue
    previous = key
    observed.append(dict(seconds=(timestamp - t0) / 1e9, reason=message.reason,
                         elapsed_ms=quality.get('elapsed_ms'),
                         candidates=quality.get('candidates', [])))
coarse, fine = collections.Counter(), collections.Counter()
elapsed = collections.defaultdict(list)
for result in observed:
    if result['candidates']:
        elapsed[result['reason']].append(result['elapsed_ms'])
    for candidate in result['candidates']:
        coarse[candidate.get('coarse', {}).get('reason')] += 1
        if 'fine' in candidate:
            fine[candidate['fine'].get('reason')] += 1
camera = list(csv.DictReader((capture / 'sensor_assets/camera_sync.csv').open()))
camera_window = []
for i, row in enumerate(camera):
    if 537 <= int(row['frame_id']) <= 546:
        row = dict(row)
        row['arrival_interval_ms'] = (int(row['arrival_ns']) - int(camera[i-1]['arrival_ns'])) / 1e6
        row['arrival_to_publish_ms'] = ((int(row['publish_ns']) - int(row['arrival_ns'])) / 1e6
                                       if int(row['publish_ns']) else None)
        row['device_tick_interval'] = int(row['dev_ts_ticks']) - int(camera[i-1]['dev_ts_ticks'])
        camera_window.append(row)
print(json.dumps(dict(
    source_capture=str(capture), read_only=True,
    note='Observed heartbeat diagnostics are not a complete solver-attempt trace; counts are not absolute accuracy or precision/recall.',
    recovery_seconds=(rows[end][0] - t0) / 1e9,
    result_counts=dict(collections.Counter(r['reason'] for r in observed)),
    candidate_coarse_reasons=dict(coarse), candidate_fine_reasons=dict(fine),
    observed_search_elapsed_ms={reason: dict(count=len(v), minimum=min(v), median=statistics.median(v),
                                           maximum=max(v), sum=sum(v)) for reason, v in elapsed.items()},
    result_timeline=[{k: v for k, v in r.items() if k != 'candidates'} for r in observed],
    camera_window=camera_window,
), indent=2, ensure_ascii=False))
