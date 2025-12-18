from setuptools import find_packages, setup

package_name = 'yolo'

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
    maintainer='junsu',
    maintainer_email='junsoo122@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone = yolo.cone:main',
            'traffic = yolo.traffic:main',
            'light_manta = yolo.traffic_light:main',
            'light_logitech = yolo.traffic_light_logitech:main',
            'check_t = yolo.check_traffic:main',
            'concat = yolo.image_concat_trafficlight:main',
        ],
    },
)
