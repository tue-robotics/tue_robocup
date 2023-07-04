#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # #  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    packages=['challenge_stickler_for_rules'],
    package_dir={'': 'src'},
    # entry_points={
    #     "console_scripts": [
    #         "carry_my_luggage = challenge_carry_my_luggage.carry_my_luggage:main",
    #     ],
    # },
)

setup(**d)
