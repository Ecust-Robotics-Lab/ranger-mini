#!/usr/bin/env python3
"""Check launch permission/model/CAN guards without executing any Node action."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import LogInfo
from launch_ros.actions import Node

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('base_launch', root / 'config/base_mini_v2.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def context(**changes):
    c = LaunchContext()
    c.launch_configurations.update(execute='false', confirmed_model='unconfirmed',
                                   control_authorized='false')
    c.launch_configurations.update(changes)
    return c


with patch.object(module.subprocess, 'run') as query:
    actions = module.prepare(context())
    assert len(actions) == 1 and isinstance(actions[0], LogInfo)
    query.assert_not_called()
    for params in ({'execute': 'true'},
                   {'execute': 'true', 'confirmed_model': 'ranger_mini_v2'}):
        try:
            module.prepare(context(**params))
            raise AssertionError('Missing permission/model did not block')
        except RuntimeError:
            pass
    query.assert_not_called()
    permitted = context(execute='true', confirmed_model='ranger_mini_v2', control_authorized='true')
    query.return_value = SimpleNamespace(stdout=json.dumps([{'flags': []}]))
    try:
        module.prepare(permitted)
        raise AssertionError('DOWN CAN did not block')
    except RuntimeError:
        pass
    query.return_value = SimpleNamespace(stdout=json.dumps([{
        'flags': ['UP'], 'linkinfo': {'info_data': {
            'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 500000}}}}]))
    actions = module.prepare(permitted)
    assert len(actions) == 1 and isinstance(actions[0], Node)
    # Intentionally never execute this action: the CAN response above is synthetic.
print('PASS: default starts nothing; model, permission and CAN guards block; '
      'authorized synthetic path only constructs an action. No driver executed.')
