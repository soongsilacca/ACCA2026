from glob import glob

import os

from setuptools import setup

package_name = 'ntrip_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_params_rclpy = ntrip_ros.test_params_rclpy:main',
            'serial_ntrip_ros = ntrip_ros.serial_ntrip_ros:main',
            'socket_ntrip = ntrip_ros.socket_ntrip_ros:main',
        ],
    },
)
