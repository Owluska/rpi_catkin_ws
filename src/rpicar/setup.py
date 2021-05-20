from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['rpicar'],
    package_dir = {'':'src'},
    requires = ['roslib', 'rospy','cv_bridge',  'std_msgs', 'sensor_msgs']
)

setup(**setup_args)
