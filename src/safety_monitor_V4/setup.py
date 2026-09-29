from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=[
        "safety_monitor",
        "safety_monitor.common",
        "safety_monitor.adapters",
        "safety_monitor.lanes",
        "safety_monitor.collision",
        "safety_monitor.health",
    ],
    package_dir={"": "src"},
))
