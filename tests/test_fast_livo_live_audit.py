"""Functional audit must not treat private success or recording completion as readiness."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace as M
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2/navigation'))
from audit_live_capture import summarize

class AuditTests(unittest.TestCase):
    def healthy(self):
        return [('/wheel/odom',M()),('/fast_livo/navigation/wheel/odom',M(header=M(stamp=M(sec=100,nanosec=0)))),
                ('/fast_livo/navigation/clock_status',M(data='{"ready":true,"error":null}')),
                ('/fast_livo/localizer/estimate',M(state=1,pose_valid=True,last_confirmed_stamp_ns=100_000_000_000)),
                ('/navigation_ready',M(data=True)),('/tf',M(transforms=[M(header=M(frame_id='map'),child_frame_id='odom'),M(header=M(frame_id='odom'),child_frame_id='base_link')])),
                ('/localization_health',M(status=[M(values=[M(key='innovation_unlocked',value='1')])]))]
    def test_observed_and_missing(self):
        result=summarize(self.healthy());self.assertEqual(result['functional_result'],'OBSERVED')
        self.assertEqual(result['physical_sync'],'NOT_VALIDATED')
        self.assertEqual(summarize(self.healthy()[:-1])['functional_result'],'INCOMPLETE_OR_REJECTED')
    def test_private_valid_does_not_override_refusal(self):
        for topic,msg in [('/navigation_ready',M(data=False)),('/fast_livo/navigation/clock_status',M(data='{"ready":false,"error":"host stepped"}')),
                          ('/localization_health',M(status=[M(values=[M(key='innovation_unlocked',value='0')])]))]:
            rows=[(t,m) for t,m in self.healthy() if t!=topic]+[(topic,msg)]
            self.assertEqual(summarize(rows)['functional_result'],'INCOMPLETE_OR_REJECTED')
    def test_wheel_regression(self):
        rows=self.healthy()+[('/fast_livo/navigation/wheel/odom',M(header=M(stamp=M(sec=99,nanosec=0))))]
        self.assertEqual(summarize(rows)['converted_wheel_regressions'],1)
        self.assertEqual(summarize(rows)['functional_result'],'INCOMPLETE_OR_REJECTED')
if __name__=='__main__':unittest.main(verbosity=2)
