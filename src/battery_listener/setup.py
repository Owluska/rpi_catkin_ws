from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['src.scripts', 'src.scripts.library'],
    package_dir = {'':'src'},
    requires = ['roslib', 'rospy']
)

setup(**d)