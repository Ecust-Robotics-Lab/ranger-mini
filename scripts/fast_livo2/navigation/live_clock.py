"""Per-session receipt anchor shared with mapping_monitor; no physical-sync claim."""
class LiveClock:
    def __init__(self, context, tolerance_ns=100_000_000):
        self.context=context
        self.tolerance_ns=tolerance_ns
        self.anchor=None
        self.host_minus_steady=None
        self.last_count=0
        self.last_sensor=None
        self.last_receipt=None
        self.error=None
        self.residual_ns=None
        self.source_anchor=None

    def set_anchor(self, anchor, source_anchor, host_ns, steady_ns):
        anchor=tuple(map(int,anchor));source_anchor=tuple(map(int,source_anchor))
        if self.anchor is not None:
            if self.anchor!=anchor or self.source_anchor!=source_anchor:
                self.fail('authoritative anchor changed; reopen session')
            return
        self.anchor=anchor;self.source_anchor=source_anchor
        self.host_minus_steady=host_ns-steady_ns

    def fail(self, reason):
        self.error=reason
        return False

    def receipts(self, data, host_ns, steady_ns):
        if data.get('context')!=self.context or data.get('schema_version')!=1:
            return False
        row=data.get('topics',{}).get('/livox/lidar',{})
        if not row.get('first_header_ns') or not row.get('count'):return False
        anchor=(int(row['first_header_ns']),int(row['first_steady_ns']))
        if self.anchor is None:
            self.anchor=anchor
            self.host_minus_steady=host_ns-steady_ns
        if self.error:return False
        if anchor!=(self.source_anchor or self.anchor):return self.fail('receipt anchor changed; reopen session')
        if abs(host_ns-steady_ns-self.host_minus_steady)>20_000_000:
            return self.fail('host clock stepped; reopen session')
        count=int(row['count']);sensor=int(row['last_header_ns']);receipt=int(row['last_steady_ns'])
        if count<self.last_count:return self.fail('receipt counter regressed')
        if count>self.last_count:
            if self.last_sensor is not None and sensor<=self.last_sensor:
                return self.fail('sensor clock regressed; reopen session')
            self.residual_ns=sensor-(self.anchor[0]+receipt-self.anchor[1])
            if abs(self.residual_ns)>self.tolerance_ns:
                return self.fail('sensor/steady divergence exceeds tolerance')
            self.last_count=count;self.last_sensor=sensor;self.last_receipt=receipt
        return True

    def ready(self, host_ns, steady_ns):
        if self.error or self.anchor is None:return False
        if abs(host_ns-steady_ns-self.host_minus_steady)>20_000_000:
            return self.fail('host clock stepped; reopen session')
        return (self.last_count>=10 and self.last_receipt-self.anchor[1]>=1_000_000_000
                and 0<=steady_ns-self.last_receipt<=500_000_000)

    def convert(self, host_stamp_ns, host_ns, steady_ns):
        if not self.ready(host_ns,steady_ns):return None
        # Convert the original sample, never replace it with callback time.
        if not 0<=host_ns-host_stamp_ns<=500_000_000:return None
        return int(host_stamp_ns)-self.host_minus_steady+self.anchor[0]-self.anchor[1]

    def status(self, host_ns, steady_ns):
        ready=self.ready(host_ns,steady_ns)
        return dict(ready=ready,error=self.error,context=self.context,anchor=self.anchor,
                    host_minus_steady_ns=self.host_minus_steady,residual_ns=self.residual_ns,
                    source_anchor=self.source_anchor,
                    tolerance_ns=self.tolerance_ns,physical_sync_verified=False,
                    method='fixed lidar receipt anchor plus host/steady offset; transport bias unmeasured')


class StartupAnchor:
    """Acquire once after a stable receipt window; never re-anchor active estimates."""
    def __init__(self):
        self.candidate=None
        self.anchor=None
        self.source_anchor=None
        self.started=None
        self.last_count=0
        self.last_sensor=None
        self.samples=0
        self.resets=0
        self.error=None

    def observe(self, row, now_ns):
        if self.anchor or self.error:return self.anchor
        if not row.get('count') or not row.get('last_header_ns'):return None
        sensor=int(row['last_header_ns']);steady=int(row['last_steady_ns']);count=int(row['count'])
        if self.started is None:self.started=now_ns
        if now_ns-self.started>10_000_000_000:
            self.error='no stable startup clock window within 10s';return None
        if not 0<=now_ns-steady<=500_000_000:return None
        if count==self.last_count:return None
        bad=(self.candidate is None or count<self.last_count or
             (self.last_sensor is not None and sensor<=self.last_sensor))
        if self.candidate:
            bad|=abs(sensor-self.candidate[0]-steady+self.candidate[1])>50_000_000
        if bad:
            self.candidate=(sensor,steady);self.samples=0;self.resets+=1
        self.last_count=count;self.last_sensor=sensor;self.samples+=1
        if self.samples>=5 and steady-self.candidate[1]>=1_000_000_000:
            self.anchor=self.candidate
            self.source_anchor=(int(row['first_header_ns']),int(row['first_steady_ns']))
        return self.anchor
