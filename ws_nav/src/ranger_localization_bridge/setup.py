from glob import glob
import os

from setuptools import setup

package_name = 'ranger_localization_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ranger_nav',
    maintainer_email='dev@example.com',
    description='Bridge Lightning-LM localization TF and Ranger wheel odometry for Nav2 (map->odom, odom->base_link, navigation_ready).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ranger_localization_bridge = ranger_localization_bridge.bridge_node:main',
        ],
    },
)
