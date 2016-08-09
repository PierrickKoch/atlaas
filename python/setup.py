import numpy
import subprocess
from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

def pkg_config(*packages, **kw):
    flag_map = {'-I': 'include_dirs', '-L': 'library_dirs', '-l': 'libraries'}
    command = ['pkg-config', '--libs', '--cflags']+list(packages)
    for token in subprocess.check_output(command).decode().split():
        kw.setdefault(flag_map.get(token[:2]), []).append(token[2:])
    kw.pop(None, None) # in case of unwanted result, eg. -Wl,-rpath,
    kw['runtime_library_dirs'] = kw['library_dirs'] # configure rpath
    return kw

extensions = [
    Extension(
        name = "atlaas._atlaas",
        sources = ["atlaas/_atlaas.pyx"],
        language = "c++",
        extra_compile_args = ["-std=c++0x", "-Wno-unused-function"],
        **pkg_config("atlaas", include_dirs=[numpy.get_include()])
    ),
]

with open('atlaas/version.py') as f:
    exec(f.read()) # get __version__

setup(name='atlaas',
      version=__version__,
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
      license='BSD',
      packages=['atlaas', 'atlaas.helpers'],
      requires=['numpy'],
      scripts=['tools/matlaas.py'],
      ext_modules=cythonize(extensions),
)
