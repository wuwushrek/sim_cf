## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# generate_distutils_setup fetches values from package.xml
# module are made available as `crazyflie_gazebo` python package
setup_args = generate_distutils_setup(
    packages=['crazyflie_gazebo'],
    package_dir={'': 'tools'},
)

setup(**setup_args)