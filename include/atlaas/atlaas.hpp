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
#include <vector>
#include <string>

#include "gdalwrap/gdal.hpp"

namespace atlaas {

std::vector<std::string> MAP_NAMES =
     {"N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "SIGMA_Z"};
enum { N_POINTS,   Z_MIN,   Z_MAX,   Z_MEAN,   SIGMA_Z,   N_RASTER};
// internal use only
enum { H_STATE=N_RASTER, N_INTERNAL};

typedef std::array<double, 2> point_xy_t;   // XY
typedef std::array<double, 3> point_xyz_t;  // XYZ
typedef std::array<double, 4> point_xyzi_t; // XYZI
typedef std::vector<point_xyz_t> points;    // PointsXYZ
typedef std::array<float, N_INTERNAL> point_info_t;
typedef std::vector<point_info_t> points_info_t;

/*
 * atlaas
 */
class atlaas {
    gdalwrap::gdal map;
    points_info_t internal;

    /**
     * fill internal from map
     */
    void _fill_internal();

public:
    /**
     * init the georeferenced map meta-data
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
        size_t width  = std::ceil(size_x / scale);
        size_t height = std::ceil(size_y / scale);
        map.set_size(width, height, N_RASTER);
        map.set_transform(utm_x, utm_y, scale, -scale);
        map.set_utm(utm_zone, utm_zone);
        map.set_custom_origin(custom_x, custom_y);
        map.names = MAP_NAMES;
        // set internal points info structure size to map (gdal) size
        internal.resize( width * height );
    }

    /**
     * init from an exisiting map
     */
    void init(std::string filepath) {
        map.load(filepath);
        _fill_internal();
    }
    void init(const gdalwrap::gdal& gmap) {
        map = gmap;
        _fill_internal();
    }

    void set_rotation(double rotation) {
        // TODO map.set_rotation(rotation);
    }

    /**
     * get a const ref on the map after updating its values
     */
    const gdalwrap::gdal& get() {
        // might want to check if update is needed ?
        update(); // on-demand update
        return map;
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
     * merge existing dtm
     */
    void merge(const gdalwrap::gdal& gmap);
};

} // namespace atlaas

#endif // ATLAAS_HPP

