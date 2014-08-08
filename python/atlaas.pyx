from libcpp cimport bool
import numpy as np
cimport numpy as np

cdef extern from "../include/atlaas/atlaas.hpp" namespace "atlaas":
    cdef cppclass atlaas:
        atlaas()
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone, bool utm_north) except +
        void merge_np(const float* cloud, size_t cloud_len1, size_t cloud_len2,
                      const double* transformation)
        void save_currents()

cdef class Atlaas:
    cdef atlaas *thisptr # hold a C++ instance which we're wrapping
    def __cinit__(self):
        self.thisptr = new atlaas()
    def __dealloc__(self):
        del self.thisptr
    def init(self, size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone, utm_north):
        self.thisptr.init(size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone, utm_north)
    def merge(self,
              np.ndarray[np.float32_t, ndim=2] C,
              np.ndarray[np.double_t,  ndim=2] T):
        assert T.size == 16 # Matrix(4,4)
        assert 3 <= C.shape[1] <= 4 # XYZ[I]
        self.thisptr.merge_np(<const float*> C.data, C.shape[0], C.shape[1],
                              <const double*> T.data)
    def save_currents(self):
        self.thisptr.save_currents()
