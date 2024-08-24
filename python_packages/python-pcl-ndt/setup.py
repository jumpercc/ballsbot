# -*- coding: utf-8 -*-
from __future__ import print_function
from collections import defaultdict
from Cython.Distutils import build_ext
from setuptools import setup, Extension

import subprocess
import sys

setup_requires = []
install_requires = [
    'Cython>=0.26.0',
]


def pkgconfig(flag):
    p = subprocess.Popen(['pkg-config', flag] + pcl_libs, stdout=subprocess.PIPE)
    stdout, _ = p.communicate()
    return stdout.decode().split()


PCL_SUPPORTED = ["-1.8", "-1.10", ""]  # in order of preference

for pcl_version in PCL_SUPPORTED:
    if subprocess.call(['pkg-config', 'pcl_common%s' % pcl_version]) == 0:
        break
else:
    print("%s: error: cannot find PCL, tried" %
          sys.argv[0], file=sys.stderr)
    for version in PCL_SUPPORTED:
        print('    pkg-config pcl_common%s' % version, file=sys.stderr)
    sys.exit(1)

pcl_libs = ["common", "filters", "geometry", "io", "registration"]
pcl_libs = ["pcl_%s%s" % (lib, pcl_version) for lib in pcl_libs]

ext_args = defaultdict(list)

for flag in pkgconfig('--cflags-only-I'):
    ext_args['include_dirs'].append(flag[2:])

ext_args['library_dirs'].append('/usr/lib')

for flag in pkgconfig('--cflags-only-other'):
    if flag.startswith('-D'):
        macro, value = flag[2:].split('=', 1)
        ext_args['define_macros'].append((macro, value))
    else:
        ext_args['extra_compile_args'].append(flag)

ext_args['extra_compile_args'].append("-std=c++17")
ext_args['library_dirs'].append("/usr/lib/x86_64-linux-gnu/")

for flag in pkgconfig('--libs-only-l'):
    if flag == "-lflann_cpp-gd":
        print(
            "skipping -lflann_cpp-gd (see https://github.com/strawlab/python-pcl/issues/29")
        continue
    ext_args['libraries'].append(flag[2:])

for flag in pkgconfig('--libs-only-L'):
    ext_args['library_dirs'].append(flag[2:])

for flag in pkgconfig('--libs-only-other'):
    ext_args['extra_link_args'].append(flag)

ext_args['include_dirs'].append("../../ros_modules/ballsbot_pose_ndt/src")

module = [
    Extension(
        "python_pcl_ndt.python_pcl_ndt",
        [
            "python_pcl_ndt.pyx",
            "../../ros_modules/ballsbot_pose_ndt/src/ndt.cpp",
            "../../ros_modules/ballsbot_pose_ndt/src/ndt_tracker.cpp"
        ],
        language="c++",
        **ext_args
    ),
]

setup(
    name='python-pcl-ndt',
    description='Python bindings for the Point Cloud Library\'s Normal Distributions Transform. using Cython.',
    version='0.3',
    author='Oleg Nurtdinov',
    author_email='jumperpro@gmail.com',
    license='BSD',
    packages=[
        "python_pcl_ndt",
    ],
    zip_safe=False,
    setup_requires=setup_requires,
    install_requires=install_requires,
    ext_modules=module,
    cmdclass={'build_ext': build_ext},
)
