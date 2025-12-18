import os
from setuptools import setup
from glob import glob

package_name = 'erp42_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='gjs',
    maintainer_email='junseonggg2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'erp42_serial = erp42_communication.erp42_serial:main',
            'test_cmd = erp42_communication.test_cmd:main',
            'steer_imu = erp42_communication.steer_imu:main',
            'compare_steer = erp42_communication.compare_steer:main',
        ],
    },
)
