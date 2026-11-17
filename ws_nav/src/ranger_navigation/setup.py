from glob import glob
import os

from setuptools import setup, find_packages
from pathlib import Path

package_name = 'ranger_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml', 'LICENSE.exact_mppi', 'THIRD_PARTY_NOTICES.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ranger_nav',
    maintainer_email='lab307@example.invalid',
    description='SE(2)/EXACT-MPPI navigation and independent final safety chain.',
    license='Apache-2.0 AND GPL-3.0',
    entry_points={
        'console_scripts': [
            'motion_gate = ranger_navigation.safety_node:main',
            'exact_navigation = ranger_navigation.coordinator:main',
        ],
    },
)
