cimport numpy as np
import numpy
from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "stdint.h":
    ctypedef unsigned long long uint64_t

cdef extern from "atlaas/atlaas.hpp" namespace "atlaas":
    cdef void tile_to_region_io(const string&, const string&) except +
    cdef void merge_io(const string&, const string&) except +
    cdef cppclass atlaas:
        atlaas()
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone, bool utm_north) except +
        void c_merge(const float* cloud, size_t cloud_len1, size_t cloud_len2,
                     const double* transformation, const double* covariance,
                     bool dump) except +
        void c_save(const string& filepath, const float* cloud,
                    size_t cloud_len1, size_t cloud_len2,
                    const double* transformation) except +
        void merge(const string& filepath, bool dump)
        void save_currents()
        void export8u(const string& filepath) except +
        void export_zmean(const string& filepath) except +
        size_t process(size_t start, size_t end)
        size_t reprocess(uint64_t last_good_pose, uint64_t time_of_fix,
                         double fixed_pose_x, double fixed_pose_y)
        void set_atlaas_path(const string& path)
        string get_atlaas_path()
        void region(const string& filepath) except +
        size_t c_closest_pcd(double x, double y, uint64_t tmax)
        string cloud_filepath(size_t seq)

def tile_to_region(fin, fout):
    tile_to_region_io(fin, fout)

def merge(fglob, fout):
    merge_io(fglob, fout)

cdef class Atlaas:
    cdef atlaas *thisptr # hold a C++ instance which we're wrapping
    def __cinit__(self):
        self.thisptr = new atlaas()
    def __dealloc__(self):
        del self.thisptr
    def init(self, size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone, utm_north=True):
        self.thisptr.init(size_x, size_y, scale, custom_x, custom_y, custom_z,
              utm_zone, utm_north)
    def merge(self,
              np.ndarray[np.float32_t, ndim=2] cloud,
              np.ndarray[np.double_t,  ndim=2] transformation,
              np.ndarray[np.double_t,  ndim=2] covariance=None, dump=True):
        if covariance is None:
            covariance = numpy.array([0]*36, dtype=numpy.float64).reshape(6, 6)
        if not covariance.size == 36:
            raise TypeError("array size must be 36, covariance: Matrix(6,6)")
        if not transformation.size == 16:
            raise TypeError("array size must be 16, transformation: Matrix(4,4)")
        if not 3 <= cloud.shape[1] <= 4:
            raise TypeError("array shape[1] must be 3 or 4, cloud: XYZ[I]")
        self.thisptr.c_merge(<const float*> cloud.data,
                             cloud.shape[0], cloud.shape[1],
                             <const double*> transformation.data,
                             <const double*> covariance.data, dump)
    def save(self, filepath,
        np.ndarray[np.float32_t, ndim=2] cloud,
        np.ndarray[np.double_t,  ndim=2] transformation):
        if not transformation.size == 16:
            raise TypeError("array size must be 16, transformation: Matrix(4,4)")
        if not 3 <= cloud.shape[1] <= 4:
            raise TypeError("array shape[1] must be 3 or 4, cloud: XYZ[I]")
        self.thisptr.c_save(filepath, <const float*> cloud.data,
                            cloud.shape[0], cloud.shape[1],
                            <const double*> transformation.data)
    def merge_file(self, filepath, dump=False):
        self.thisptr.merge(filepath, dump)
    def save_currents(self):
        self.thisptr.save_currents()
    def export8u(self, filepath):
        self.thisptr.export8u(filepath)
    def export_zmean(self, filepath):
        self.thisptr.export_zmean(filepath)
    def process(self, start=0, end=2**32):
        return self.thisptr.process(start, end)
    def reprocess(self, last_good_pose, time_of_fix, fixed_pose_x, fixed_pose_y):
        return self.thisptr.reprocess(last_good_pose, time_of_fix, fixed_pose_x,
                                      fixed_pose_y)
    def set_atlaas_path(self, path):
        self.thisptr.set_atlaas_path(path)
    def get_atlaas_path(self):
        return self.thisptr.get_atlaas_path()
    def region(self, filepath):
        self.thisptr.region(filepath)
    def closest_pcd(self, x, y, tmax):
        """
        tmax most recent PCD bound in milliseconds,
             should be (time.time()*1000 - T)
        """
        return self.thisptr.c_closest_pcd(x, y, tmax)
    def cloud_filepath(self,seq):
        return self.thisptr.cloud_filepath(seq)