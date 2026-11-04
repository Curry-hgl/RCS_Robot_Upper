from setuptools import find_packages, setup
from glob import glob
package_name = 'rcs_vision'
setup(name=package_name, version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/test', glob('test/test_*.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml'))], install_requires=['setuptools'], zip_safe=True, entry_points={"console_scripts": ["test = rcs_vision.test_runner:main", "rcs_vision = rcs_vision.rcs_vision_node:main"]})
