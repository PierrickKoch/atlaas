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

#include <memory> // unique_ptr C++11
#include <ctime> // std::time
#include <string>
#include <sstream> // ostringstream

#include <gdalwrap/gdal.hpp>
#include <atlaas/common.hpp>


#define DYNAMIC_MERGE

namespace atlaas {

// init tile-path from the environment variable ATLAAS_PATH
static const std::string ATLAAS_PATH = getenv("ATLAAS_PATH", ".");

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
    cells_info_t internal; // to merge dyninter
    cells_info_t gndinter; // ground info for vertical/flat unknown state
    cells_info_t dyninter; // to merge point cloud
    vbool_t      vertical; // altitude state (vertical or not)
    float        variance_factor;

    /**
     * current location in the tiles frame
     */
    map_id_t current;

    /**
     * need update I/O ?
     */
    bool map_sync;

    /**
     * {x,y} map size
     */
    size_t width;
    size_t height;

    /**
     * tiles data
     */
    int sw; // tile-width
    int sh; // tile-height
    std::unique_ptr<atlaas> tile;

    /**
     * time base
     */
    std::time_t time_base;

    /**
     * fill internal from map
     */
    void _fill_internal();

    /**
     * Seconds since the base time.
     *
     * Since we'll store datas as float32, time since epoch would give
     * something like `1.39109e+09`, we substract a time_base
     * (to be set using `atlaas::set_time_base(time_t)`).
     */
    float get_reference_time() {
        return std::time(NULL) - time_base;
    }

public:
    /**
     * init the georeferenced map meta-data
     * we recommend width and height being 3 times the range of the sensor
     * for a Velodyne, we recommend 120x120m @ 0.1m/pixel resolution.
     *
     * @param size_x    width  in meters
     * @param size_y    height in meters
     * @param scale     size of a pixel in meters
     * @param custom_x  custom X origin in UTM in meters
     * @param custom_y  custom Y origin in UTM in meters
     * @param custom_z  custom Z origin in UTM in meters
     * @param utm_zone  UTM zone
     * @param utm_north is UTM north?
     */
    void init(double size_x, double size_y, double scale,
              double custom_x, double custom_y, double custom_z,
              int utm_zone, bool utm_north = true) {
        width  = std::ceil(size_x / scale);
        height = std::ceil(size_y / scale);
        map.set_size(N_RASTER, width, height);
        map.set_transform(custom_x, custom_y, scale, -scale);
        map.set_utm(utm_zone, utm_north);
        map.set_custom_origin(custom_x, custom_y, custom_z);
        map.names = MAP_NAMES;
        set_time_base( std::time(NULL) );
        // set internal points info structure size to map (gdal) size
        internal.resize( width * height );
        map_sync = true;
        current = {{0,0}};
        // load maplets if any
        // works if we init with the same parameters,
        // even if the robot is at a different pose.
        sw = width  / 3; // tile-width
        sh = height / 3; // tile-height
        tile = std::move(std::unique_ptr<atlaas>(new atlaas));
        tile->map.copy_meta(map, sw, sh);
        tile->internal.resize(sw * sh);
        tile_load(-1, -1);
        tile_load(-1,  0);
        tile_load(-1,  1);
        tile_load( 0, -1);
        tile_load( 0,  0);
        tile_load( 0,  1);
        tile_load( 1, -1);
        tile_load( 1,  0);
        tile_load( 1,  1);
#ifdef DYNAMIC_MERGE
        // atlaas used for dynamic merge
        dyninter.resize( width * height );
        vertical.resize( width * height );
        gndinter.resize( width * height );
        variance_factor = 3.0;
#endif
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

    std::string tilepath(int x, int y) const {
        std::ostringstream oss;
        oss << ATLAAS_PATH << "/atlaas." << x << "x" << y << ".tif";
        return oss.str();
    }

    void set_time_base(std::time_t base) {
        time_base = base;
        map.metadata["TIME"] = std::to_string(time_base);
    }

    void set_variance_factor(float factor) {
        variance_factor = factor;
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
    const cells_info_t& get_internal() const {
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
    void merge(const points& cloud, cells_info_t& infos);

    /**
     * transform, merge, slide, save, load tiles
     */
    void merge(points& cloud, const matrix& transformation);

    /**
     * slide, save, load tiles
     */
    void slide_to(double robx, double roby);
    void tile_load(int sx, int sy);
    void tile_save(int sx, int sy) const;
    void save_currents() const {
        tile_save(-1, -1);
        tile_save(-1,  0);
        tile_save(-1,  1);
        tile_save( 0, -1);
        tile_save( 0,  0);
        tile_save( 0,  1);
        tile_save( 1, -1);
        tile_save( 1,  0);
        tile_save( 1,  1);
    }

    /**
     * dynamic merge of cloud in custom frame
     */
    void dynamic(const points& cloud);

    /**
     * compute real variance and return the mean
     */
    float variance_mean(cells_info_t& inter);

    /**
     * merge existing dtm for dynamic merge
     */
    void merge();
    void merge(cell_info_t& dst, const cell_info_t& src);
};

} // namespace atlaas

#endif // ATLAAS_HPP

