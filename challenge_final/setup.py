#!/usr/bin/env python

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(
    # #  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    packages=['challenge_final'],
    package_dir={'': 'src'}
)

setup(**d)
