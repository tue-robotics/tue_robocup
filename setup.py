#!/usr/bin/env python

from setuptools import setup

setup(  name='robot_skills',
        version='1.0',
        author='Sjoerd van den Dries',
        author_email='s.v.d.dries@tue.nl',
        url='http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Robot_skills',
        license='LICENSE.txt',
        description='High-level interface to robot components',
        long_description="Wrappers for each robot sub system comprise a high-level robot interface.",
        install_requires=["singledispatch", "ipdb"]
)
