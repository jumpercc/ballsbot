# -*- coding: utf-8 -*-
from __future__ import print_function
from collections import defaultdict
from Cython.Distutils import build_ext
from setuptools import setup, Extension

import subprocess

setup_requires = []
install_requires = [
    'Cython>=0.29.0',
]


def pkgconfig(flag):
    p = subprocess.Popen(['pkg-config', flag], stdout=subprocess.PIPE)
    stdout, _ = p.communicate()
    return stdout.decode().split()


ext_args = defaultdict(list)

for flag in pkgconfig('--cflags-only-I'):
    ext_args['include_dirs'].append(flag[2:])
ext_args['include_dirs'].append("/usr/include/eigen3")

ext_args['library_dirs'].append('/usr/lib')

for flag in pkgconfig('--cflags-only-other'):
    if flag.startswith('-D'):
        macro, value = flag[2:].split('=', 1)
        ext_args['define_macros'].append((macro, value))
    else:
        ext_args['extra_compile_args'].append(flag)

ext_args['extra_compile_args'].append("-std=c++17")
ext_args['library_dirs'].append("/usr/lib/aarch64-linux-gnu")
ext_args['library_dirs'].append("/usr/lib/x86_64-linux-gnu/")
ext_args['library_dirs'].append("./ballsbot_localization")

for flag in pkgconfig('--libs-only-l'):
    ext_args['libraries'].append(flag[2:])

for flag in pkgconfig('--libs-only-L'):
    ext_args['library_dirs'].append(flag[2:])

for flag in pkgconfig('--libs-only-other'):
    ext_args['extra_link_args'].append(flag)

module = [
    Extension(
        "ballsbot_localization.ballsbot_localization",
        [
            "ballsbot_localization.pyx",
            "src/grid.cpp", "src/point_cloud.cpp", "src/geometry.cpp", "src/free_distances.cpp",
        ],
        language="c++",
        **ext_args
    ),
]

# sudo rm -rf ballsbot_localization.cpp build && python3 setup.py build_ext
# sudo python3 setup.py install -f

setup(
    name='ballsbot_localization',
    description='Python bindings for ballsbot_localization functions.',
    version='0.2',
    author='Oleg Nurtdinov',
    author_email='jumperpro@gmail.com',
    license='BSD',
    packages=[
        "ballsbot_localization",
    ],
    zip_safe=False,
    setup_requires=setup_requires,
    install_requires=install_requires,
    ext_modules=module,
    cmdclass={'build_ext': build_ext},
)
