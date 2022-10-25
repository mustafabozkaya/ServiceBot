from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['robot_control'],
    scripts=['/scripts'],
    package_dir={'': 'src'}
)

setup(**setup_args)
