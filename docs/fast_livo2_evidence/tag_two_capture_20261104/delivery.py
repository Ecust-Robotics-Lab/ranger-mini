#!/usr/bin/env python3
"""Audit software timestamp association and source provenance for both new captures."""
import csv
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('consistency',ROOT/'scripts/fast_livo2/tags/analyze_capture_consistency.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)

def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: existing capture software delivery audit');return 0
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    results=[]
    for key,name in [('turn','fast-livo-tag140_turn_03-bk8_pjyy'),('straight','fast-livo-tag140_straight_04-ryzykq2l')]:
        capture=ROOT/'bags'/name
        connection=a.common.open_readonly(capture/'raw/raw_0.db3')
        try:
            tid=connection.execute("SELECT id FROM topics WHERE name='/livox/lidar'").fetchone()[0]
            lidar=[a.common.stamp(deserialize_message(data,PointCloud2)) for data, in connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id',(tid,))]
        finally:connection.close()
        with (capture/'sensor_assets/camera_sync.csv').open() as f:camera=list(csv.DictReader(f))
        with (capture/'sensor_assets/timeshare_writer.csv').open() as f:writer=list(csv.DictReader(f))
        timing,links=a.analyze_sync(camera,writer,lidar)
        sequences=[int(r['publication_sequence_number']) for r in writer if r['publication_sequence_number']]
        timing['dds_publication_sequence_gap_count']=sum(b!=a+1 for a,b in zip(sequences,sequences[1:]))
        timing['publisher_gids']=sorted(set(r['publisher_gid_hex'] for r in writer))
        timing['dds_lost_total_max']=max(int(r['dds_lost_total']) for r in writer if r['dds_lost_total'])
        timing['capture']=name
        (HERE/(key+'_delivery.json')).write_text(json.dumps(timing,indent=2)+'\n')
        (HERE/(key+'_camera_writer_links.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in links))
        results.append(timing)
    (HERE/'delivery_summary.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps([{k:r[k] for k in ['capture','camera_frames','recorded_lidar_scans','recorded_lidar_scans_missing_from_writer','dds_publication_sequence_gap_count','dds_lost_total_max','anomalies']} for r in results],indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
