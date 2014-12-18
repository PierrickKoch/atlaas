import subprocess
from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

def pkg_config(*packages, **kw):
    flag_map = {'-I': 'include_dirs', '-L': 'library_dirs', '-l': 'libraries'}
    command = ['pkg-config', '--libs', '--cflags']+list(packages)
    for token in subprocess.check_output(command).decode().split():
        kw.setdefault(flag_map.get(token[:2]), []).append(token[2:])
    return kw

pkg_config_dict = pkg_config("atlaas")

extensions = [
    Extension(
        name = "atlaas",
        sources = ["atlaas.pyx"],
        language = "c++",
        extra_compile_args = ["-std=c++0x", "-Wno-unused-function"],
        # configure rpath
        runtime_library_dirs = pkg_config_dict['library_dirs'],
        **pkg_config_dict
    ),
]

setup(ext_modules = cythonize(extensions))
