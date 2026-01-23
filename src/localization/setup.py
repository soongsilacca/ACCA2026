from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*.pcd')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Localization package for ERP42 robot including wheel odometry and EKF sensor fusion',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gps_to_odometry = localization.gps_to_odometry:main',
            'ekf_global_initializer = localization.ekf_global_initializer:main',
            'ekf_local_initializer = localization.ekf_local_initializer:main',
            'imu_nwu_adapter = localization.imu_nwu_adapter:main',
            'odom_map_republisher = localization.odom_map_republisher:main',
            'wheel_odometry_erp = localization.wheel_odometry_erp:main',
            'rmse_analyzer = localization.rmse_analyzer:main',
            'pcd_map_publisher = localization.pcd_map_publisher:main',
            'fastlio_odometry_adapter = localization.fastlio_odometry_adapter:main',
        ],
    },
)
