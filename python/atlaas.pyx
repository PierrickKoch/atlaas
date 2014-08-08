from libcpp cimport bool
from libcpp.vector cimport vector

cdef extern from "../include/atlaas/atlaas.hpp" namespace "atlaas":
    cdef cppclass atlaas:
        atlaas()
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone, bool utm_north) except +
        void merge_np(vector[vector[float]], const vector[double])
        void save_currents()

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
    def merge(self, cloud, transformation):
        self.thisptr.merge_np(cloud, transformation)
    def save_currents(self):
        self.thisptr.save_currents()
