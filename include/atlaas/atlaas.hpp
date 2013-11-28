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
#include <map>
#include <vector>
#include <string>

#include <gdalwrap/gdal.hpp>

namespace atlaas {

std::vector<std::string> MAP_NAMES =
     {"N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "SIGMA_Z"};
enum { N_POINTS,   Z_MIN,   Z_MAX,   Z_MEAN,   SIGMA_Z,   N_RASTER};
// internal use only
enum { H_STATE=N_RASTER, N_INTERNAL};

typedef std::array<double, 2> point_xy_t;   // XY (for UTM frame)
typedef std::array<float,  3> point_xyz_t;  // XYZ (custom frame)
typedef std::vector<point_xyz_t> points;    // PointsXYZ
typedef std::array<float, N_INTERNAL> point_info_t;
typedef std::vector<point_info_t> points_info_t;
typedef std::map<std::string, std::string> map_str_t;
typedef std::array<int, 2> point_id_t; // submodels location

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
    map_str_t submodels;
    point_id_t current;

    /**
     * need update I/O ?
     */
    bool map_sync;

    /**
     * fill internal from map
     */
    void _fill_internal();

public:
    /**
     * init the georeferenced map meta-data
     * we recomment width and height being 3 times the range of the sensor
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
        size_t width  = std::ceil(size_x / scale);
        size_t height = std::ceil(size_y / scale);
        map.set_size(N_RASTER, width, height);
        map.set_transform(utm_x, utm_y, scale, -scale);
        map.set_utm(utm_zone, utm_zone);
        map.set_custom_origin(custom_x, custom_y);
        map.names = MAP_NAMES;
        // set internal points info structure size to map (gdal) size
        internal.resize( width * height );
        map_sync = true;
        current = {{0,0}};
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
     * merge and slide, save, load submodels
     */
    void merge(const points& cloud, double robx, double roby);

    /**
     * slide, save, load submodels
     */
    void slide_to(int subx, int suby);

    /**
     * merge existing dtm
     */
    void merge(const atlaas& from);
};

} // namespace atlaas

#endif // ATLAAS_HPP

