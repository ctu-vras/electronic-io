from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['electronic_io', 'electronic_io.devices', 'electronic_io.virtual_pins'],
    package_dir={'': 'src'}
)

setup(**d)
