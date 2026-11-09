"""Replay acceptance must reject stale readiness, duplicate TF, commands and latching."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2/navigation'))
from replay_localization import accept_result


class ReplayTests(unittest.TestCase):
    def good(self):
        return dict(valid_samples=20,ready_samples=19,tf_checks=18,tf_translation_error_max_m=1e-12,tf_rotation_error_max_rad=0.,
                    end_stale_ready=False,gap_ready_samples=[False],post_gap_ready_recovered=True,
                    bridge_health={'innovation_unlocked':'1'},dynamic_tf_publishers=1,
                    velocity_publishers={'/cmd_vel':0,'/base/safe_cmd_vel':0})

    def test_healthy_replay_and_failed_recovery(self):
        r=self.good();self.assertTrue(accept_result(r,True))
        r['post_gap_ready_recovered']=False
        self.assertFalse(accept_result(r,True))
        self.assertTrue(accept_result(r,False))

    def test_private_success_does_not_override_bridge_latch(self):
        r=self.good();r['bridge_health']['innovation_unlocked']='0'
        self.assertFalse(accept_result(r,False))

    def test_tf_ownership_commands_and_stale_readiness(self):
        for changes in [dict(dynamic_tf_publishers=2),dict(end_stale_ready=True),
                        dict(velocity_publishers={'/cmd_vel':1}),dict(gap_ready_samples=[True]),
                        dict(tf_translation_error_max_m=.5),dict(tf_rotation_error_max_rad=.1)]:
            r=self.good();r.update(changes);self.assertFalse(accept_result(r,True))


if __name__=='__main__':unittest.main(verbosity=2)
