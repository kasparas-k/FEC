from distutils.command.build import build
from pathlib import Path
from setuptools import setup
from setuptools.command.build_ext import build_ext
import subprocess


class CMakeBuild(build_ext):
    def run(self):
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)
        
        subprocess.check_call(['cmake', str(Path(__file__).parent)], cwd=build_temp)
        subprocess.check_call(['make'], cwd=build_temp)

        for file in build_temp.glob("*.so"):
            dst_path = Path('fec') / file.name
            self.copy_file(str(file), str(dst_path))


class CustomBuild(build):
    def run(self):
        self.run_command('build_ext')
        build.run(self)


setup(
    name='fec',
    version='0.1',
    packages=['fec'],
    package_data={
        'fec': ['*.so'],  # Include the compiled shared library in the package
    },
    include_package_data=True,
    install_requires=['numpy', 'pybind11[global]'],
    cmdclass={
        'build_ext': CMakeBuild,
        'build': CustomBuild
    },
    description='Fast Euclidean Clustering by Cao et al. (2022) C++ bindings for Python',
    url='https://github.com/kasparas-k/FEC',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Programming Language :: C++',
        'Operating System :: OS Independent',
    ],
)
