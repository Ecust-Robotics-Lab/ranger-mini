#!/usr/bin/env python3
"""Resolve runtime defaults without sourcing ROS or starting any processes."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                      'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import main

if __name__ == '__main__':
    main()
