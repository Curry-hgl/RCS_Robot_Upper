from setuptools import find_packages, setup
package_name = 'rcs_ptl'
<<<<<<< HEAD
setup(name=package_name, version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])], install_requires=['setuptools'], zip_safe=True)
=======
setup(name=package_name, version='0.1.0', packages=find_packages(where="src"), package_dir={"": "src"}, data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])], install_requires=['setuptools'], zip_safe=True)
>>>>>>> c3d5228 (FSM与msg初步移植)
