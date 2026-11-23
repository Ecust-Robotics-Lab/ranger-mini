# Navigation workbench dependency notices

Runtime dependencies are installed into `.runtime/navigation_workbench_venv`;
versions are recorded in `requirements.lock.txt`. No third-party library source
is copied into the workbench frontend and no external CDN is used. Installed
wheel `*.dist-info/licenses/` and license metadata are retained by pip.

| Dependency | License from installed metadata |
| --- | --- |
| aiohttp 3.14.5 | Apache-2.0 AND MIT |
| aiohappyeyeballs 2.7.1 | PSF-2.0 |
| aiosignal, async-timeout, frozenlist, multidict, propcache, yarl | Apache-2.0 |
| attrs | MIT |
| idna | BSD-3-Clause |
| typing_extensions | PSF-2.0 |

Playwright/Chromium are development-only browser QA assets installed under
`.runtime/`; `requirements.dev.lock.txt` captures the Python tooling. Playwright
is Apache-2.0; its downloaded browser includes the browser's own notices.

ROS, NumPy, PyYAML, Pillow, SciPy and the existing SE(2)/EXACT-MPPI integration
retain the repository's original notices and dependency provenance. This module
does not relicense the upstream GPL integration. The browser service is local
software; no source or recording is uploaded to Sites or other hosting.
