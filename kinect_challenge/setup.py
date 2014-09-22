from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['kinect_challenge'],
    scripts=['scripts/mapper.py','scripts/navigator.py','scripts/msbmio_svs.py','scripts/msbmio_client.py'],
    package_dir={'': 'src'}
)

setup(**d)
