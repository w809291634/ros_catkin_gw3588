from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
 
d = generate_distutils_setup(
    packages=['obj_detection_rk3399'],
    package_dir={'detection': 'marm_visual_inspection'}
)

setup(**d)
