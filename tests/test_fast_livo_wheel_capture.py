"""Wheel source selection and exact CAN configuration without hardware access."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2'))
import wheel_capture as wheel


class WheelTests(unittest.TestCase):
    def test_default_driver_is_receive_only(self):
        command=wheel.driver_command()
        self.assertIn('receive_only:=true',command)
        self.assertIn('publish_odom_tf:=false',command)

    def test_external_feedback_reused_and_stale_never_replaced(self):
        good={'type':'nav_msgs/msg/Odometry','reliability':'RELIABLE'}
        sample={'stamp_ns':10**9,'frame':'odom','child':'base_link'}
        self.assertEqual(wheel.wheel_source([good],sample,1_100_000_000),'external')
        for pubs,msg in [([good,good],sample),([good],None),([good],dict(sample,child='other'))]:
            with self.assertRaises(RuntimeError):wheel.wheel_source(pubs,msg,1_100_000_000)
        with self.assertRaises(RuntimeError):wheel.wheel_source([good],sample,2*10**9)
        self.assertEqual(wheel.wheel_source([],None,0),'owned_receive_only')

    def test_up_can_preserved_and_wrong_bitrate_refused(self):
        before={'flags':['UP'],'linkinfo':{'info_data':{'state':'ERROR-ACTIVE','bittiming':{'bitrate':500000}}}}
        with patch.object(wheel.subprocess,'run') as run:
            self.assertFalse(wheel.prepare_can(before)['enabled_by_capture']);run.assert_not_called()
            before['linkinfo']['info_data']['bittiming']['bitrate']=1000000
            with self.assertRaises(RuntimeError):wheel.prepare_can(before)
            run.assert_not_called()

    def test_down_can_uses_only_exact_approved_noninteractive_command(self):
        down={'flags':[]};up={'flags':['UP'],'linkinfo':{'info_data':{'state':'ERROR-ACTIVE','bittiming':{'bitrate':500000}}}}
        with patch.object(wheel.subprocess,'run',return_value=SimpleNamespace(returncode=0,stderr='')) as run,patch.object(wheel,'can_state',return_value=up):
            self.assertTrue(wheel.prepare_can(down)['enabled_by_capture'])
            self.assertEqual(run.call_args.args[0],['sudo','-n','/usr/sbin/ip','link','set','can0','up','type','can','bitrate','500000'])
        with patch.object(wheel.subprocess,'run',return_value=SimpleNamespace(returncode=1,stderr='password required')):
            with self.assertRaisesRegex(RuntimeError,'sudo配置'):wheel.prepare_can(down)

    def test_external_source_does_not_touch_can_or_other_sessions(self):
        with patch.object(wheel,'discover',return_value={'source':'external'}) as discover,patch.object(wheel,'prepare_can') as configure,patch.object(wheel,'check_other_drivers') as drivers:
            self.assertEqual(wheel.prepare()['source'],'external');configure.assert_not_called();drivers.assert_not_called()


if __name__=='__main__':unittest.main(verbosity=2)
