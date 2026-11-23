"""Compute readiness and bounded operator reset health are separate from motion permission."""
import math


def navigation_compute_available(status,steady_now):
    worker=status.get('worker') or {}
    if worker.get('error'):return False
    if worker.get('ready'):return True
    started=status.get('reset_started_steady')
    return (status.get('state')=='RESETTING' and isinstance(started,(int,float)) and
            math.isfinite(started) and 0<=steady_now-started<=35.)
