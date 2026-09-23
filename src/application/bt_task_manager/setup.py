#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['bt_task_manager','bt_task_manager'],
    package_dir={'': 'src'},
    scripts=['scripts/controller_node','scripts/grasp_node']
)

setup(**setup_args)