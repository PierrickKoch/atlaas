/*
 * atlaas.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-10-08
 * license: BSD
 */
#ifndef ATLAAS_HPP
#define ATLAAS_HPP

#include <array> // C++11
#include <memory> // unique_ptr C++11
#include <map>
#include <vector>
#include <string>
#include <sstream> // ostringstream
#include <sys/stat.h> // stat

#include <gdalwrap/gdal.hpp>

namespace atlaas {

std::vector<std::string> MAP_NAMES =
     {"N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "SIGMA_Z"};
enum { N_POINTS,   Z_MIN,   Z_MAX,   Z_MEAN,   SIGMA_Z,   N_RASTER};
// internal use only
enum { H_STATE=N_RASTER, N_INTERNAL};

typedef std::array<double, 2> point_xy_t;   // XY (for UTM frame)
typedef std::array<float,  3> point_xyz_t;  // XYZ (custom frame)
typedef std::array<double, 16> matrix;      // transformation matrix
typedef std::array<double, 6> pose6d;       // yaw,pitch,roll,x,y,z
typedef std::vector<point_xyz_t> points;    // PointsXYZ
typedef std::array<float, N_INTERNAL> point_info_t;
typedef std::vector<point_info_t> points_info_t;
typedef std::map<std::string, std::string> map_str_t;
typedef std::array<int, 2> point_id_t; // submodels location
typedef std::vector<point_id_t> points_id_t;

/**
 * atlaas
 */
class atlaas {
    /**
     * I/O data model
     */
    gdalwrap::gdal map;

    /**
     * internal data model
     */
    points_info_t internal;

    /**
     * history of saved submodels, to load them back
     */
    point_id_t current;

    /**
     * need update I/O ?
     */
    bool map_sync;

    /**
     * {x,y} map size
     */
    size_t width;
    size_t height;
    int sw; // sub-width
    int sh; // sub-height
    std::unique_ptr<atlaas> sub;
    std::unique_ptr<atlaas> dyn;

    /**
     * fill internal from map
     */
    void _fill_internal();

public:
    /**
     * init the georeferenced map meta-data
     * we recommend width and height being 3 times the range of the sensor
     * for a Velodyne, we recommend 90x90m @ 0.1m/pixel resolution.
     *
     * @param size_x    width  in meters
     * @param size_y    height in meters
     * @param scale     size of a pixel in meters
     * @param custom_x  custom X origin in UTM in meters
     * @param custom_y  custom Y origin in UTM in meters
     * @param utm_x     UTM (X) origin in meters
     * @param utm_y     UTM (Y) origin in meters
     * @param utm_zone  UTM zone
     * @param utm_north is UTM north?
     */
    void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double utm_x, double utm_y,
              int utm_zone, bool utm_north = true) {
        width  = std::ceil(size_x / scale);
        height = std::ceil(size_y / scale);
        map.set_size(N_RASTER, width, height);
        map.set_transform(utm_x, utm_y, scale, -scale);
        map.set_utm(utm_zone, utm_zone);
        map.set_custom_origin(custom_x, custom_y);
        map.names = MAP_NAMES;
        // set internal points info structure size to map (gdal) size
        internal.resize( width * height );
        map_sync = true;
        current = {{0,0}};
        // load maplets if any
        // works if we init with the same parameters,
        // even if the robot is at a different pose.
        sw = width  / 3; // sub-width
        sh = height / 3; // sub-height
        sub = std::move(std::unique_ptr<atlaas>(new atlaas));
        sub->map.copy_meta(map, sw, sh);
        sub->internal.resize(sw * sh);
        sub_load(-1, -1);
        sub_load(-1,  0);
        sub_load(-1,  1);
        sub_load( 0, -1);
        sub_load( 0,  0);
        sub_load( 0,  1);
        sub_load( 1, -1);
        sub_load( 1,  0);
        sub_load( 1,  1);
        // atlaas used for dynamic merge
        dyn = std::move(std::unique_ptr<atlaas>(new atlaas));
        dyn->internal.resize( width * height );
    }

    /**
     * init from an exisiting map
     */
    void init(std::string filepath) {
        map.load(filepath);
        _fill_internal();
    }
    void init(const gdalwrap::gdal& gmap) {
        map = gmap; // copy (!)
        _fill_internal();
    }

    void set_rotation(double rotation) {
        // TODO map.set_rotation(rotation);
    }

    /**
     * get a const ref on the map after updating its values
     */
    const gdalwrap::gdal& get() {
        if (not map_sync)
            update(); // on-demand update
        return map;
    }

    /**
     * get a const ref on the map without updating its values
     */
    const gdalwrap::gdal& get_unsynced_map() const {
        return map;
    }

    /**
     * get a const ref on the internal data (aligned points)
     * used for our local planner message conversion
     */
    const points_info_t& get_internal() const {
        return internal;
    }

    /**
     * Save Z_MEAN as a grayscale image (for display)
     */
    void export8u(const std::string& filepath) {
        get().export8u(filepath, Z_MEAN);
    }

    /**
     * update internal -> map
     */
    void update();

    /**
     * merge point-cloud in internal structure
     */
    void merge(const points& cloud);

    /**
     * transform, merge, slide, save, load submodels
     */
    void merge(points& cloud, const matrix& transformation);

    /**
     * slide, save, load submodels
     */
    void slide_to(double robx, double roby);
    void sub_load(int sx, int sy);
    void sub_save(int sx, int sy) const;
    void save_currents() const {
        sub_save(-1, -1);
        sub_save(-1,  0);
        sub_save(-1,  1);
        sub_save( 0, -1);
        sub_save( 0,  0);
        sub_save( 0,  1);
        sub_save( 1, -1);
        sub_save( 1,  0);
        sub_save( 1,  1);
    }

    /**
     * reset all cells for dynamic merge
     */
    void reset();

    /**
     * dynamic merge of cloud in custom frame
     */
    void dynamic(const points& cloud);

    /**
     * merge existing dtm for dynamic merge
     */
    void merge(const atlaas& from);
};

/**
 * Returns weither the file exists or not on POSIX systems (use <sys/stat.h>)
 */
inline bool file_exists(const std::string& name) {
    struct stat buffer;
    return ( stat(name.c_str(), &buffer) == 0 );
}

inline std::string sub_name(int x, int y) {
    std::ostringstream oss;
    oss << "atlaas." << x << "x" << y << ".tif";
    return oss.str();
}

/**
 * Transformation helpers
 */

/**
 * Pose6d(yaw,pitch,roll,x,y,z) -> Matrix[16]
 */
inline matrix pose6d_to_matrix(double yaw, double pitch, double roll,
                               double x, double y, double z) {
    matrix mat;
    double ca, sa, cb, sb, cg, sg;

    ca = cos(yaw);   sa = sin(yaw);
    cb = cos(pitch); sb = sin(pitch);
    cg = cos(roll);  sg = sin(roll);

    mat[0]  = ca*cb;
    mat[1]  = ca*sb*sg - sa*cg;
    mat[2]  = ca*sb*cg + sa*sg;
    mat[3]  = x;

    mat[4]  = sa*cb;
    mat[5]  = sa*sb*sg + ca*cg;
    mat[6]  = sa*sb*cg - ca*sg;
    mat[7]  = y;

    mat[8]  = -sb;
    mat[9]  = cb*sg;
    mat[10] = cb*cg;
    mat[11] = z;

    mat[12] = 0.0;
    mat[13] = 0.0;
    mat[14] = 0.0;
    mat[15] = 1.0;

    return mat;
}

/**
 * Pose6d(yaw,pitch,roll,x,y,z) -> Matrix[16]
 */
inline matrix pose6d_to_matrix(const pose6d& pose) {
    return pose6d_to_matrix(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
}

/**
 * Matrix[16] -> Pose6d(yaw,pitch,roll,x,y,z)
 */
inline pose6d matrix_to_pose6d(const matrix& mat) {
    double yaw,pitch,roll,x,y,z;

    double d = sqrt(mat[0]*mat[0] + mat[4]*mat[4]);

    if (fabs(d) > 1e-10) {
        yaw  = atan2(mat[4], mat[0]);
        roll = atan2(mat[9], mat[10]);
    } else {
        yaw  = atan2(-mat[1], mat[5]);
        roll = 0.0;
    }
    pitch = atan2(-mat[8], d);

    x = mat[3];
    y = mat[7];
    z = mat[11];

    return {{yaw,pitch,roll,x,y,z}};
}

} // namespace atlaas

#endif // ATLAAS_HPP

