## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cpr_mover_controller'],
    package_dir={'': 'scripts'},
    requires=['std_msgs', 'rospy', 'geometry_msgs', 'sensor_msgs']
)

setup(**setup_args)