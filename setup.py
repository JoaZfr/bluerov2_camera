from setuptools import setup
import os
from glob import glob

package_name = 'bluerov2_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'model'), glob('model/best.pt')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for publishing stereo camera data from Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Camera #
            'camera_manager = bluerov2_camera.camera_manager:main',
            'camera_viewer = bluerov2_camera.camera_viewer:main',
        ],
    },
)