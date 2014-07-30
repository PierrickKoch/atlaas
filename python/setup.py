from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

# setup(ext_modules = cythonize("*.pyx"))
extensions = [
    Extension(
        name = "atlaas",
        sources = [
            #"atlaas.pxd",
            "atlaas.pyx",
            "../src/atlaas.cpp",
            "../src/slide.cpp",
        ],
        language = "c++",
        libraries = ["atlaas", "gdalwrap",],
        library_dirs = ["/home/pkoch/devel/lib"],
        include_dirs = ["/home/pkoch/devel/include"],
        extra_compile_args = ["-std=c++0x"],
    ),
]

setup(ext_modules = cythonize(extensions))
