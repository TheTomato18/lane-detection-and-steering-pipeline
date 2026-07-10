import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lane_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nageeb Moin',
    maintainer_email='thetomato18@pm.me',
    description='Lane detection and steering pipeline as a ROS2 node graph',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = lane_pipeline.camera_node:main',
            'perception_node = lane_pipeline.perception_node:main',
            'control_node = lane_pipeline.control_node:main',
            'visualizer_node = lane_pipeline.visualizer_node:main',
        ],
    },
)
