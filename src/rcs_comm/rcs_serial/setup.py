from setuptools import find_packages, setup
package_name = 'rcs_serial'
setup(name=package_name, version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])], install_requires=['setuptools', 'pyserial'], entry_points={'console_scripts': ['list_ports = rcs_serial.rcs_serial_ports:main', 'rcs_serial_loopback = rcs_serial.rcs_serial_loopback:main', 'serial_bridge = rcs_serial.rcs_serial_bridge:main']}, zip_safe=True)
