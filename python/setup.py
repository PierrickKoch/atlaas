import subprocess
from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

# python setup.py build_ext --inplace

libraries = ["atlaas", "gdalwrap"]
flags = subprocess.check_output(['pkg-config', '--cflags-only-I', ' '.join(libraries)])
include_dirs = [flag[2:] for flag in flags.split()]
flags = subprocess.check_output(['pkg-config', '--libs-only-L', ' '.join(libraries)])
library_dirs = [flag[2:] for flag in flags.split()]

extensions = [
    Extension(
        name = "atlaas",
        sources = [
            "atlaas.pyx",
            "../src/atlaas.cpp",
            "../src/slide.cpp",
        ],
        language = "c++",
        extra_compile_args = ["-std=c++0x"],
        libraries = libraries,
        library_dirs = library_dirs,
        include_dirs = include_dirs,
    ),
]

setup(ext_modules = cythonize(extensions))
