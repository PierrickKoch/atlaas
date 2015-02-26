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
pkg_config_dict.pop(None, None) # in case of unwanted result, eg. -Wl,-rpath,

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

setup(name='atlaas',
      version='0.1',
      description='Python bindings for Atlaas (using Cython)',
      author='Pierrick Koch',
      author_email='pierrick.koch@laas.fr',
      url='https://github.com/pierriko/atlaas',
      classifiers=[
        'Development Status :: 4 - Beta',
        'License :: OSI Approved :: BSD License',
        'Operating System :: POSIX',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering',
      ],
      requires=['numpy'],
      ext_modules=cythonize(extensions),
)
