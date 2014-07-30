import subprocess
from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

# python setup.py build_ext --inplace

def pkg_config(*libraries):
    arg = ' '.join(libraries)
    flags = subprocess.check_output(['pkg-config', '--cflags-only-I', arg])
    include_dirs = [flag[2:] for flag in flags.split()]
    flags = subprocess.check_output(['pkg-config', '--libs-only-L', arg])
    library_dirs = [flag[2:] for flag in flags.split()]
    return {'libraries': libraries,
            'include_dirs': include_dirs,
            'library_dirs': library_dirs}

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
        **pkg_config("atlaas", "gdalwrap")
    ),
]

setup(ext_modules = cythonize(extensions))
