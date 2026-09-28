from glob import glob
import os

from setuptools import setup

package_name = 'ranger_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ranger_nav',
    maintainer_email='lab307@example.invalid',
    description='Stage 6 navigation integration and velocity safety chain.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motion_gate = ranger_navigation.motion_gate:main',
            'obstacle_filter = ranger_navigation.obstacle_filter:main',
        ],
    },
)
