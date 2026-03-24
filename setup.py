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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iota',
    maintainer_email='lp05180@student.uni-lj.si',
    description='A semester project involving TurtleBot4 and ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ring_detector = megatron.ring_detector:main'
            'face_detector = megatron.face_detector:main'
        ],
    },
)
