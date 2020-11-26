# -*- coding: utf-8 -*-
from __future__ import print_function
from collections import defaultdict
from Cython.Distutils import build_ext
from setuptools import setup, Extension

import subprocess
import sys

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
ext_args['include_dirs'].append("/usr/include/opencv4")

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
ext_args['library_dirs'].append("./ballsbot_cpp")

for flag in pkgconfig('--libs-only-l'):
    ext_args['libraries'].append(flag[2:])
ext_args['libraries'].append('opencv_core')
ext_args['libraries'].append('opencv_videoio')

for flag in pkgconfig('--libs-only-L'):
    ext_args['library_dirs'].append(flag[2:])

for flag in pkgconfig('--libs-only-other'):
    ext_args['extra_link_args'].append(flag)

module = [
    Extension(
        "ballsbot_cpp.ballsbot_cpp",
        [
            "ballsbot_cpp.pyx",
            "ballsbot/geometry.cpp", "ballsbot/point_cloud.cpp", "ballsbot/grid.cpp",
            "ballsbot/cam_detector.cpp",
        ],
        language="c++",
        runtime_library_dirs=[  # FIXME
            '/usr/local/lib/python3.6/dist-packages/ballsbot_cpp-0.1-py3.6-linux-aarch64.egg/ballsbot_cpp',
            '/usr/lib/aarch64-linux-gnu',
        ],
        **ext_args
    ),
]

setup(
    name='ballsbot-cpp',
    description='Python bindings for ballsbot cpp functions.',
    version='0.1',
    author='Oleg Nurtdinov',
    author_email='j@jumper.cc',
    license='BSD',
    packages=[
        "ballsbot_cpp",
    ],
    zip_safe=False,
    setup_requires=setup_requires,
    install_requires=install_requires,
    ext_modules=module,
    cmdclass={'build_ext': build_ext},
    package_data={'': ['libuffssd.so', 'ssd_coco_labels.txt', 'ssd_relu6.uff']},
    include_package_data=True,
    libraries=[
        ("uffssd", {'sources': ["ballsbot/cam_detector.cpp"]}),
        ("opencv_core", {'sources': ["ballsbot/cam_detector.cpp"]}),
        ("opencv_videoio", {'sources': ["ballsbot/cam_detector.cpp"]}),
    ],
)
