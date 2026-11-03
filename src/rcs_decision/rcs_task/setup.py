from setuptools import find_packages, setup
from glob import glob

package_name = 'rcs_task'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'gcs_task = rcs_task.gcs_node:main',
          'fsm_example = rcs_task.example:main',
          'task_example = rcs_task.node:main',
      ]})
