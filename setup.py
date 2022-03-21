#!/usr/bin/env python3

# ! Do not manually invoke this setup.py. Use catkin instead

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["video_recorder"],
    package_dir={"video_recorder": "ros/src/video_recorder"}
)

setup(**setup_args)
