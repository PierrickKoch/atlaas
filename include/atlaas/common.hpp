/*
 * common.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2014-03-18
 * license: BSD
 */
#ifndef ATLAAS_COMMON_HPP
#define ATLAAS_COMMON_HPP

#include <array> // C++11
#include <cstdlib> // std::getenv
#include <cmath> // std::sqrt
#include <vector>
#include <string>
#include <sys/stat.h> // stat, file_exists

namespace atlaas {

static const std::vector<std::string> MAP_NAMES =
     {"N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "VARIANCE", "TIME", "DIST_SQ"};
enum { N_POINTS,   Z_MIN,   Z_MAX,   Z_MEAN,   VARIANCE,   TIME,   DIST_SQ,   N_RASTER};

typedef std::array<double, 2> point_xy_t;   // XY (for UTM frame)
typedef std::array<float,  3> point_xyz_t;  // XYZ (custom frame)
typedef std::array<double, 16> matrix;      // transformation matrix
typedef std::array<double, 6> pose6d;       // yaw,pitch,roll,x,y,z
typedef std::vector<point_xyz_t> points;    // PointsXYZ
typedef std::array<float, N_RASTER> cell_info_t;
typedef std::vector<cell_info_t> cells_info_t;
typedef std::vector<bool> vbool_t; // altitude state (vertical or not)
typedef std::array<int, 2> map_id_t; // tiles location

/**
 * System helpers
 */

/**
 * getenv with default value if the variable is not set
 */
inline std::string getenv(const std::string& name, const std::string& def) {
    const char* value = std::getenv( name.c_str() );
    return value ? value : def;
}

/**
 * Returns whether the file exists or not on POSIX systems (use <sys/stat.h>)
 */
inline bool file_exists(const std::string& name) {
    struct stat buffer;
    return ( stat(name.c_str(), &buffer) == 0 );
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

/**
 * Matrix[16] -> Pose3d(x,y,z)
 */
inline point_xyz_t matrix_to_point(const matrix& mat) {
    return {{ static_cast<float>(mat[3]),
              static_cast<float>(mat[7]),
              static_cast<float>(mat[11]) }};
}

/**
 * Apply the transformation matrix to the point cloud (in place)
 *
 * with a 6 digits floating-point precision, at a decimeter resolution,
 * it is safe to use this method up to 99 km from the origin.
 * ( cf. std::numeric_limits<float>::digits10 )
 */
inline void transform(points& cloud, const matrix& tr) {
    float x,y,z;
    for (auto& point : cloud) {
        x = point[0];
        y = point[1];
        z = point[2];
        point[0] = (x * tr[0]) + (y * tr[1]) + (z * tr[2])  + tr[3];
        point[1] = (x * tr[4]) + (y * tr[5]) + (z * tr[6])  + tr[7];
        point[2] = (x * tr[8]) + (y * tr[9]) + (z * tr[10]) + tr[11];
    }
}

/**
 * Euclidian distance (squared)
 *
 * usefull to compare a set of points (faster)
 */
inline float distance_sq(const point_xyz_t& pA, const point_xyz_t& pB) {
    float x = pA[0] - pB[0];
    float y = pA[1] - pB[1];
    float z = pA[2] - pB[2];
    return x*x + y*y + z*z;
}

/**
 * Euclidian distance
 */
template <class Point>
inline float distance(const Point& pA, const Point& pB) {
    return std::sqrt(distance_sq(pA, pB));
}

} // namespace atlaas

#endif // ATLAAS_COMMON_HPP

