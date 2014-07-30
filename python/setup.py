import subprocess
from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

# python setup.py build_ext --inplace

def pkg_config(*packages, **kw):
    flag_map = {'-I': 'include_dirs', '-L': 'library_dirs', '-l': 'libraries'}
    for token in subprocess.check_output(['pkg-config', '--libs', '--cflags']+list(packages)).decode().split():
        kw.setdefault(flag_map.get(token[:2]), []).append(token[2:])
    return kw

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
        **pkg_config("atlaas")
    ),
]

setup(ext_modules = cythonize(extensions))
