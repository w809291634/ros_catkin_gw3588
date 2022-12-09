from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
 
d = generate_distutils_setup(
    packages=['src'],
    package_dir={'arm_tcp_client_controller': 'marm_controller'}
)

setup(**d)
