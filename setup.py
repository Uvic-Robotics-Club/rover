# -*- coding: utf-8 -*-
"""
Created on Sat Jan 06 16:30:25 2018

@author: joell
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rover'],
    package_dir={'': 'src'},
)

setup(**setup_args)