"""Capture-bound offline epoch translation. Transport/motion fit is not physical sync."""
import hashlib
import json
from pathlib import Path
import numpy as np


class CaptureClock:
    def __init__(self, analysis, capture):
        self.capture=Path(capture).resolve()
        folder=Path(analysis)
        info=json.loads((folder/'analysis.json').read_text())
        metadata=self.capture/'raw/metadata.yaml'
        if Path(info['source_bag']).resolve()!=metadata.parent:
            raise ValueError('clock mapping belongs to another capture')
        if hashlib.sha256(metadata.read_bytes()).hexdigest()!=info['source_metadata_sha256']:
            raise ValueError('capture metadata changed')
        self.offset_ns=int(info['candidate_sensor_to_host_offset_ns'])
        self.metadata_sha256=info['source_metadata_sha256']
        self.analysis_sha256=hashlib.sha256((folder/'analysis.json').read_bytes()).hexdigest()
        fits=info['sign_corrected_angular_fits']
        self.window_lag_spread_s=abs(fits['first_half']['lag_seconds']-fits['second_half']['lag_seconds'])

    def host_to_sensor(self, ns):
        if isinstance(ns,np.ndarray):
            return ns.astype(np.int64)-np.int64(self.offset_ns)
        return int(ns)-self.offset_ns

    def to_dict(self):
        return {'schema_version':1,'kind':'offline_capture_clock_mapping','capture':str(self.capture),
            'source_metadata_sha256':self.metadata_sha256,'analysis_sha256':self.analysis_sha256,
            'convention':'t_host_ns = t_sensor_ns + offset_ns','offset_ns':self.offset_ns,
            'window_lag_spread_s':self.window_lag_spread_s,'physical_sync_verified':False,
            'offline_only':True,'validity_scope':'this capture only; never reuse across device boots/sessions',
            'original_headers_modified':False,'range_policy':'wheel history brackets only; no extrapolation'}
