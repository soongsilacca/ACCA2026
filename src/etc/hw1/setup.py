from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hw1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob(os.path.join('launch',"*launch.py")))
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='bum',
    maintainer_email='bumwhale333c@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_node = hw1.publish_node:main',
            'subscribe_node = hw1.subscribe_node:main',

        ],
    },
)
