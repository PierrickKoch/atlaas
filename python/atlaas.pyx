# import dereference and increment operators
from cython.operator cimport dereference as deref, preincrement as inc

cdef extern from "<array>" namespace "std":
    cdef cppclass array[T, N]:
        cppclass iterator:
            T operator*()
            iterator operator++()
            bint operator==(iterator)
            bint operator!=(iterator)
        array()
        T& operator[](int)
        T& at(int)
        iterator begin()
        iterator end()

from libcpp cimport bool
from libcpp.vector cimport vector

cdef extern from *:
    ctypedef int int_parameter
    int_parameter _N4 = "4"
    int_parameter _N16 = "16"

cdef extern from "../include/atlaas/atlaas.hpp" namespace "atlaas":
    ctypedef vector[array[double, _N4]] points
    ctypedef array[double, _N16] matrix
    cdef cppclass atlaas:
        atlaas()
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone, bool utm_north) except +
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone) except +
        void merge(points& cloud, const matrix& transformation)

cdef class PyAtlaas:
    cdef atlaas *thisptr # hold a C++ instance which we're wrapping
    def __cinit__(self):
        self.thisptr = new atlaas()
    def __dealloc__(self):
        del self.thisptr
    def init(self, size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone, utm_north):
        self.thisptr.init(size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone, utm_north)
    def init(self, size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone):
        self.thisptr.init(size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone)
    #def merge(self, cloud, transformation):
    #    self.thisptr.merge(cloud, transformation)

