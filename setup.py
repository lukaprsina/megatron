import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'megatron'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'waypoints'),
            glob(os.path.join('waypoints', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luka',
    maintainer_email='lp05180@student.uni-lj.si',
    description='A semester project involving TurtleBot4 and ROS2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'face_detector = megatron.face_detector:main',
            'ring_detector = megatron.ring_detector:main',
            'controller = megatron.controller:main',
            'perception_visualizer = megatron.perception_visualizer:main',
        ],
    },
)
