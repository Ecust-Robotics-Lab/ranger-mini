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
    description='Backend-independent localization bridge and Lightning adapter for Nav2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ranger_localization_bridge = ranger_localization_bridge.bridge_node:main',
            'lightning_adapter = ranger_localization_bridge.lightning_adapter:main',
            'fast_livo_adapter = ranger_localization_bridge.fast_livo_adapter:main',
        ],
    },
)
