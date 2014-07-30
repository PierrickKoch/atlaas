# distutils: language = c++
# distutils: sources = ../src/atlaas.cpp

from libcpp cimport bool
from libcpp cimport vector

cdef extern from "../include/atlaas/atlaas.hpp" namespace "atlaas":
    cdef cppclass atlaas:
        atlaas()
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone, bool utm_north) except +
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone) except +
        # void merge(points& cloud, const matrix& transformation)
