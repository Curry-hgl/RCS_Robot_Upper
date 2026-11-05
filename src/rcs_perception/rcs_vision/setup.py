from setuptools import find_packages, setup
from glob import glob
from pathlib import Path
from distutils.command.install_data import install_data


class ConfigInstall(install_data):
    """配置链接到源码，普通构建后也可直接修改。"""

    def run(self):
        configs = [
            (Path(source).resolve(), Path(self.install_dir) / target / Path(source).name)
            for target, sources in self.data_files
            if target.endswith('/config')
            for source in sources
        ]
        if not self.dry_run:
            for source, destination in configs:
                if destination.is_symlink():
                    destination.unlink()
        super().run()
        if not self.dry_run:
            for source, destination in configs:
                destination.unlink()
                destination.symlink_to(source)


package_name = 'rcs_vision'
setup(cmdclass={'install_data': ConfigInstall}, name=package_name, version='0.1.0', packages=find_packages(), data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/test', glob('test/test_*.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml'))], install_requires=['setuptools'], zip_safe=True, entry_points={"console_scripts": ["test = rcs_vision.test_runner:main", "rcs_vision = rcs_vision.rcs_vision_node:main"]})
