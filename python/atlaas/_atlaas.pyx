import numpy as np
cimport numpy as cnp
from libc.stdlib cimport free

cnp.import_array()

def tile_to_region(fin, fout):
    tile_to_region_io(fin, fout)

def merge(fglob, fout, compress=False):
    merge_io(fglob, fout, compress)

def save(filepath,
    cnp.ndarray[cnp.float32_t, ndim=2] cloud,
    cnp.ndarray[cnp.double_t,  ndim=2] transformation):
    """ Use for test only (inefficient)
    since the point-cloud is copied 2 times:
    (float*) numpy.array -> atlaas::points -> pcl::PointCloud
    """
    if not transformation.size == 16:
        raise TypeError("array size must be 16, transformation: Matrix(4,4)")
    if not 3 <= cloud.shape[1] <= 4:
        raise TypeError("array shape[1] must be 3 or 4, cloud: XYZ[I]")
    c_save(filepath, <const float*> cloud.data,
                     cloud.shape[0], cloud.shape[1],
                     <const double*> transformation.data)

def load(filepath):
    """ Use for test only (inefficient)
    since the point-cloud is copied 3 times:
    pcl::PointCloud -> atlaas::points -> float* -> numpy.array
    """
    cdef size_t length = 0
    cdef double[16] transformation
    cdef float* cloud = c_load(filepath, length, transformation)
    cdef float[:,:] view_cd = <float[:length, :4]> cloud
    np_cd = np.asarray(view_cd).copy()
    free(cloud) # free cloud from malloc
    np_tr = np.array([transformation[i] for i in range(16)]).reshape(4, 4)
    return (np_tr, np_cd)

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
              cnp.ndarray[cnp.float32_t, ndim=2] cloud,
              cnp.ndarray[cnp.double_t,  ndim=2] transformation,
              cnp.ndarray[cnp.double_t,  ndim=2] covariance=None, dump=True):
        if covariance is None:
            covariance = np.array([0]*36, dtype=np.float64).reshape(6, 6)
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
    def cloud_filepath(self, seq):
        return self.thisptr.cloud_filepath(seq)
    def pcd_xy(self):
        return self.thisptr.pcd_xy_str()
    def pcd_map(self):
        return self.thisptr.pcd_map_str()
    def pcd_time(self):
        return self.thisptr.pcd_time_str()
    def pcd_overlap(self, id):
        return self.thisptr.get_pcd_overlap_str(id)
    def closest_pcd_id(self, stamp):
        """ Get the closest PCD identifier to a given time-stamp
        :param stamp: time-stamp in milliseconds
        :type stamp: int
        :returns: pcd identifier
        :rtype: int
        """
        return self.thisptr.get_closest_pcd_id(stamp)
    def set_use_swap(self, value):
        self.thisptr.set_use_swap(value)
    def set_variance_threshold(self, threshold):
        self.thisptr.set_variance_threshold(threshold)
