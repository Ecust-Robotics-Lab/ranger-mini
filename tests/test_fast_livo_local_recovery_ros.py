#!/usr/bin/env python3
"""Actual C2 node, localhost synthetic geometry: outage recovery and prior scope."""
import json
import unittest
import test_fast_livo_relocalizer_ros as c2
c1=c2.c1
c1.EVIDENCE=c1.ROOT/'docs/fast_livo2_evidence/failure_fix_20261109/ros'
c1.EVIDENCE.mkdir(parents=True,exist_ok=True)
c1.EXECUTABLE=c1.ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'

class Recovery(c2.C2Ros):
    # Only run these targeted cases, not inherited historical cases.
    def launch(self):
        process=self.spawn([str(c1.EXECUTABLE),'--ros-args','--params-file',str(c1.ROOT/'config/fast_livo2/localizer.yaml'),
            '--params-file',str(c1.ROOT/'config/fast_livo2/relocalizer.yaml'),'-p','archive_path:='+str(self.archive),
            '-p','compatibility_path:='+str(self.compatibility),'-p','online_scheduling:=true'], 'recovery_node')
        self.wait(lambda:self.private and self.private[-1].map_uuid and json.loads(self.private[-1].quality_json).get('inputs_ready'))
        return process

    def test_short_outages_retain_request_and_stationary_recovers(self):
        self.launch();self.send(wheel=False);good=self.search_to_valid()
        # Freeze actual robot pose while advancing sample/clock time.
        frozen=self.wheel_pose(good.latest_scan_stamp_ns)
        self.wheel_pose=lambda ns:frozen
        for gap in [.3,.6,1.]:
            with self.subTest(gap=gap):
                old_request=self.private[-1].initialization_request_id
                self.ns+=int(gap*1e9);self.pump(.15)
                if gap>=.6:self.assertFalse(self.private[-1].pose_valid)
                for _ in range(6):
                    result=self.send(wheel=False)
                    if result.pose_valid:break
                self.assertTrue(result.pose_valid,result.reason)
                self.assertEqual(result.initialization_request_id,old_request)
                self.assertLessEqual(json.loads(result.quality_json)['sample_age_seconds'],.5)
        self.assertFalse(self.node.get_publishers_info_by_topic('/cmd_vel'))

    def test_long_outage_returns_to_global_search(self):
        self.launch();self.send(wheel=False);good=self.search_to_valid()
        self.ns+=3_000_000_000;self.pump(.15)
        self.assertFalse(self.private[-1].pose_valid)
        observed=[]
        for _ in range(5):observed.append(self.send(wheel=False).reason)
        self.assertTrue(any('global_' in x or 'query' in x or 'retry' in x for x in observed),observed)

# Avoid duplicating inherited acceptance cases.
for name in list(c2.C2Ros.__dict__):
    if name.startswith('test_'):setattr(Recovery,name,None)
if __name__=='__main__':unittest.main(verbosity=2)
