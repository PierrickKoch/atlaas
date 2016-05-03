from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "stdint.h":
    ctypedef unsigned long long uint64_t

cdef extern from "atlaas/atlaas.hpp" namespace "atlaas":
    cdef void tile_to_region_io(const string&, const string&) except +
    cdef void merge_io(const string&, const string&, bool) except +
    cdef void c_save(const string& filepath, const float* cloud,
                     size_t cloud_len1, size_t cloud_len2,
                     const double* transformation) except +
    cdef float* c_load(const string& filepath, size_t& length,
                       double* transformation) except +
    cdef cppclass atlaas:
        atlaas()
        void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone, bool utm_north) except +
        void c_merge(const float* cloud, size_t cloud_len1, size_t cloud_len2,
                     const double* transformation, const double* covariance,
                     bool dump) except +
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
        string pcd_xy_str()
        string pcd_map_str()
        string pcd_time_str()
        string get_pcd_overlap_str(size_t id)
        size_t get_closest_pcd_id(uint64_t)
        void set_use_swap(bool value)
        void set_variance_threshold(float threshold)
