"""Latency audit must measure new samples, not repeatedly published heartbeats."""
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys
import pytest
pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ws_nav/src/ranger_navigation'),str(ROOT/'ws_nav/src/ranger_localization_bridge')]
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from ranger_localization_msgs.msg import LocalizationEstimate
spec=importlib.util.spec_from_file_location('latency_audit',ROOT/'scripts/analyze_exact_input_latency.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def test_heartbeat_does_not_count_as_new_sample_or_extra_transport_delay(tmp_path):
    bag=tmp_path/'bag';(bag/'raw').mkdir(parents=True)
    (bag/'recorder_result.json').write_text(json.dumps({'exit':0}))
    con=sqlite3.connect(bag/'raw/raw_0.db3')
    con.execute('create table topics(id integer,name text,type text)')
    con.execute('create table messages(timestamp integer,topic_id integer,data blob)')
    for i,n,t in [(1,'/clock','rosgraph_msgs/msg/Clock'),(2,'/fast_livo/odometry','nav_msgs/msg/Odometry'),
                  (3,'/localization/estimate','ranger_localization_msgs/msg/LocalizationEstimate')]:
        con.execute('insert into topics values(?,?,?)',(i,n,t))
    def add(t,i,m):con.execute('insert into messages values(?,?,?)',(t,i,serialize_message(m)))
    for sample,receipt in [(1,10_000_000_000),(2,11_000_000_000)]:
        c=Clock();c.clock.sec=sample;add(receipt,1,c)
        m=Odometry();m.header.stamp.sec=sample;add(receipt+10_000_000,2,m)
        loc=LocalizationEstimate();loc.header.stamp.sec=sample;loc.state=loc.VALID
        add(receipt+30_000_000,3,loc);add(receipt+90_000_000,3,loc)
    con.commit();con.close()
    result=audit.audit(bag,10_000_000_000,12_000_000_000)
    public=result['stages']['/localization/estimate']
    assert public['new_samples']==2
    assert public['new_sample_receipt_interval_ms']['p50']==1000.
    # No private estimate was recorded: do not invent a paired transport delay.
    assert result['links']['/fast_livo/localizer/estimate -> /localization/estimate']['matched_samples']==0


def test_audit_rejects_unfinalized_recording(tmp_path):
    (tmp_path/'recorder_result.json').write_text(json.dumps({'exit':None}))
    with pytest.raises(ValueError,match='finalized'):
        audit.audit(tmp_path,0,1)
