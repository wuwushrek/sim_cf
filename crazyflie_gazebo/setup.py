# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# generate_distutils_setup fetches values from package.xml
# module are made available as `crazyflie_gazebo` python package
# setup_args = generate_distutils_setup(
#     packages=['crazyflie_gazebo_tools'],
#     package_dir={'crazyflie_gazebo_tools': 'tools/crazyflie_gazebo_tools'},
# )
setup_args = generate_distutils_setup(
    packages=['crazyflie_gazebo', 
    'crazyflie_gazebo.tools'
    ],
    package_dir={'': 'tools',
                #  'crazyflie_gazebo.tools': 'tools/crazyflie_gazebo/tools/',
                 },
)

setup(**setup_args)
