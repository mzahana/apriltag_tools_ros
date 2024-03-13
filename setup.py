from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_tools_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('launch/*.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khaled Gabr',
    maintainer_email='khaledgabr77@gmail.com',
    description='This ROS 2 package is designed to provide tools and utilities for working with AprilTags in ROS-based robotics applications',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'apriltag_detections_to_pose = apriltag_tools_ros.apriltag_detections_to_pose:main'
        ],
    },
)
