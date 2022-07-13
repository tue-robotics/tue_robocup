#!/usr/bin/env python3
#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(packages=["challenge_clean_the_table"], package_dir={"": "src"})

setup(**d)
