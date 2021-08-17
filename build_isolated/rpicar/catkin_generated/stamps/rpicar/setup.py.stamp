from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['src.scripts', 'src.scripts.drivers', 'src.scripts.library', 'src.scripts.data'],
    package_dir = {'':'src'},
    requires = ['roslib', 'rospy','cv_bridge', 'std_msgs', 'sensor_msgs', 'nav_msgs']
)

setup(**setup_args)
