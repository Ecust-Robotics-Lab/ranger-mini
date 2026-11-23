"""Driver execution telemetry: SDK submissions are not physical delivery proof."""
import json
import numpy as np


class ExecutionFeedback:
    def __init__(self):
        self.report=None
        self.received=None
        self.session=None
        self.retired=set()

    def update(self,text,host_now,steady_now):
        try:
            value=json.loads(text)
            if value['schema_version']!=1:raise ValueError('execution_schema')
            age=host_now-value['host_stamp_ns']*1e-9
            if not 0<=age<=.25:raise ValueError('execution_age')
            if not isinstance(value['session'],str) or not value['session']:raise ValueError('execution_session')
            if value['session'] in self.retired:return 'retired'
            for name in ('requested_body','submitted_sdk'):
                vector=np.asarray(value[name],dtype=float)
                if vector.shape!=(3,) or not np.isfinite(vector).all():raise ValueError('execution_velocity')
            for name in ('feedback_ready','receive_only'):
                if not isinstance(value[name],bool):raise ValueError('execution_flags')
            if value['mode'] not in (0,1,2,255) or not isinstance(value['reason'],str):raise ValueError('execution_mode')
            if not all(np.isfinite(value[k]) for k in ('command_age_s','submission_age_s','feedback_age_s','mode_age_s','system_age_s')):
                raise ValueError('execution_command_age')
            changed=self.session is not None and self.session!=value['session']
            if changed:
                self.retired.add(self.session)
                if len(self.retired)>128:raise ValueError('execution_session_capacity')
            self.session=value['session']
            self.report,self.received=value,steady_now-age
            return 'session_changed' if changed else 'accepted'
        except (KeyError,ValueError,TypeError,OverflowError):
            self.report,self.received=None,None
            return 'invalid'

    def fresh(self,now):
        return self.report is not None and self.received is not None and 0<=now-self.received<=.25

    def ready(self,now,mode):
        if not self.fresh(now):return False
        r=self.report
        age=now-self.received
        return (r['feedback_ready'] and not r['receive_only'] and r['mode']==int(mode)
                and 0<=r['command_age_s'] and r['command_age_s']+age<=.1
                and 0<=r['submission_age_s'] and r['submission_age_s']+age<=.1
                and all(0<=r[k] and r[k]+age<=.25 for k in ('feedback_age_s','mode_age_s','system_age_s'))
                and r['reason'] in ('tracking','steering_alignment','zero_command'))

    def waiting(self,now,mode):
        return self.ready(now,mode) and self.report['reason']=='steering_alignment'

    def snapshot(self,now):
        return dict(fresh=self.fresh(now),driver=self.report)
