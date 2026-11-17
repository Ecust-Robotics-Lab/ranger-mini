#!/usr/bin/env python3
"""Experiment-local adaptation of the existing managed mapping entry."""
import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'scripts/fast_livo2'))
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'ws_nav/src/ranger_localization_bridge'))
# scripts/mapping_session.py is a different (general robot) entry. Load the
# FAST implementation by exact path, independent of PYTHONPATH ordering.
_spec = importlib.util.spec_from_file_location('p0_fast_mapping_session', ROOT/'scripts/fast_livo2/mapping_session.py')
mapping = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mapping)
import record_session
from session_manager import Component


def configure():
    if getattr(mapping, '_p0_configured', False):
        return
    mapping._p0_configured = True
    # Match current online recording's camera executable AND library prefix.
    # The general mapping entry is left untouched.
    mapping.sensor_specs = lambda cfg, vis: record_session.capture_specs(cfg, vis, 8765)
    mapping.sensor_environment = record_session.sensor_environment
    original = mapping.components

    def components(cfg, args, folder):
        items = original(cfg, args, folder)
        observer = Component('closure_observer', [sys.executable, str(HERE / 'observer.py'),
                            '--folder', str(folder)], depends=('probe',),
                            resource='fast_livo_closure_observer')
        items.insert(1, observer)
        for item in items:
            if item.name in ('backend', 'receipts'):
                item.depends += ('closure_observer',)
        return items

    mapping.components = components
    mapping.EXTRA_TOPICS = [*mapping.EXTRA_TOPICS, '/wheel/odom']


if __name__ == '__main__':
    configure()
    try:
        raise SystemExit(mapping.main())
    except (RuntimeError, OSError, ValueError) as exc:
        print('EXPERIMENT MAP ERROR: ' + str(exc), file=sys.stderr, flush=True)
        raise SystemExit(2)
