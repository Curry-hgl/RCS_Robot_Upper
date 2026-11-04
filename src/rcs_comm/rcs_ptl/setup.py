from setuptools import find_packages, setup
from glob import glob
package_name = 'rcs_ptl'
setup(name=package_name, version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/test', glob('test/test_*.py'))], install_requires=['setuptools'], entry_points={'console_scripts': ['test = rcs_ptl.test_runner:main']}, zip_safe=True)
