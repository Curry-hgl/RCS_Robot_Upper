from setuptools import find_packages, setup

package_name = 'rcs_task'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml'])],
<<<<<<< HEAD
      install_requires=['setuptools'], zip_safe=True)
=======
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'fsm_example = rcs_task.example:main',
          'task_example = rcs_task.node:main',
      ]})
>>>>>>> c3d5228 (FSM与msg初步移植)
