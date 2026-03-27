import os
from setuptools import setup
from glob import glob

package_name = 'path_plan_cone'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/cubic_spline_planner.py']),
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
            'path_cone = path_plan_cone.path_cone:main',
            'path_cone_mpc = path_plan_cone.path_cone_mpc:main',
        ],
    },
)
