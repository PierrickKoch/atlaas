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

#define PCD_VOXEL_SIZE 0.05f
#define PCD_DIST_SQ 400.0f

namespace atlaas {

// init tile-path from the environment variable ATLAAS_PATH
static const std::string ATLAAS_PATH = getenv("ATLAAS_PATH", ".");

inline std::string tilepath(int x, int y) {
    std::ostringstream oss;
    oss << ATLAAS_PATH << "/atlaas." << x << "x" << y << ".tif";
    return oss.str();
}

inline std::string pcdpath(size_t seq) {
    std::ostringstream oss;
    oss << ATLAAS_PATH << "/pcl." << seq << ".pcd";
    return oss.str();
}

/**
 * atlaas
 */
class atlaas {
    size_t pcd_seq;
    /**
     * I/O data model
     */
    gdalwrap::gdal meta;
    mutable gdalwrap::gdal tile;

    /**
     * internal data model
     */
    cells_info_t internal; // to merge dyninter
    cells_info_t gndinter; // ground info for vertical/flat unknown state
    cells_info_t dyninter; // to merge point cloud
    float        variance_threshold;
    point_xy_t   sensor_xy;

    /**
     * position of the current most North-West tile (0, 0)
     * in the global tile frame
     */
    map_id_t current;

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

    /**
     * time base
     */
    std::time_t time_base;

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
        pcd_seq = 0;
        width  = std::ceil(size_x / scale);
        height = std::ceil(size_y / scale);
        meta.set_size(width, height); // does not change the container
        meta.set_transform(custom_x, custom_y, scale, -scale);
        meta.set_utm(utm_zone, utm_north);
        meta.set_custom_origin(custom_x, custom_y, custom_z);
        set_time_base( std::time(NULL) );
        // set internal points info structure size to map (gdal) size
        internal.resize( width * height );
        current = {{0,0}};
        // load tiles if any
        // works if we init with the same parameters,
        // even if the robot is at a different pose.
        sw = width  / 3; // tile-width
        sh = height / 3; // tile-height

        tile.copy_meta_only(meta);
        tile.names = MAP_NAMES;
        tile.set_size(N_RASTER, sw, sh);

        tile_load(0, 0);
        tile_load(0, 1);
        tile_load(0, 2);
        tile_load(1, 0);
        tile_load(1, 1);
        tile_load(1, 2);
        tile_load(2, 0);
        tile_load(2, 1);
        tile_load(2, 2);

        // atlaas used for dynamic merge
        dyninter.resize( width * height );
        gndinter.resize( width * height );
        variance_threshold = 0.05;
    }

    void set_time_base(std::time_t base) {
        time_base = base;
        meta.metadata["TIME"] = std::to_string(time_base);
    }

    void set_variance_threshold(float threshold) {
        variance_threshold = threshold;
    }

    /**
     * get a const ref on the map without updating its values
     */
    const gdalwrap::gdal& get_meta() const {
        return meta;
    }

    /**
     * get a const ref on the internal data (aligned points)
     * used for our local planner message conversion
     */
    const cells_info_t& get_internal() const {
        return internal;
    }

    /**
     * merge point-cloud in internal structure
     */
    void rasterize(const points& cloud, cells_info_t& infos) const;

    /**
     * transform, merge, slide, save, load tiles
     */
    void merge(points& cloud, const matrix& transformation, bool dump = true);

    /**
     * write pcd file
     */
    void write_pcd(const points& cloud, const matrix& transformation) {
        write_pcd(pcdpath(pcd_seq++), cloud, transformation);
    }

    /**
     * write pcd file
     */
    void write_pcd(const std::string& filepath, const points& cloud,
        const matrix& transformation, float voxel_size = PCD_VOXEL_SIZE,
        float dist_sq = PCD_DIST_SQ) const;

    /**
     * read pcd file
     */
    void read_pcd(const std::string& filepath, points& cloud,
        matrix& transformation) const;

    /**
     * merge from raw C array (using std::copy !)
     * used for numpy -> C++ interface
     * transformation must be double[16] : row-major Matrix(4,4)
     * cloud must be float[cloud_len1][cloud_len2]
     * cloud_len2 must be either 3 (XYZ) or 4 (XYZI)
     */
    void c_merge(const float* cloud, size_t cloud_len1, size_t cloud_len2,
                 const double* transformation) {
        matrix tr;
        points cd( cloud_len1 );
        std::copy(transformation, transformation + 16, tr.begin());
        for (size_t i = 0; i < cloud_len1; i++)
            std::copy(cloud+i*cloud_len2, cloud+(i+1)*cloud_len2, cd[i].begin());
        merge(cd, tr);
    }

    size_t process_pcd(size_t start = 0, size_t end = std::numeric_limits<size_t>::max()) {
        points cloud;
        matrix transformation = {{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 }};
        std::string filepath = pcdpath(start);
        while ( start <= end ) {
            if ( ! file_exists(filepath) )
                break;
            read_pcd(filepath, cloud, transformation);
            merge(cloud, transformation, false);
            filepath = pcdpath(start++);
        }
        return start;
    }

    /**
     * slide, save, load tiles
     */
    bool slide();
    void do_slide() {
        while ( slide() );
    }

    void tile_load(int sx, int sy);
    void tile_save(int sx, int sy) const;
    void save_currents() const {
        tile_save(0, 0);
        tile_save(0, 1);
        tile_save(0, 2);
        tile_save(1, 0);
        tile_save(1, 1);
        tile_save(1, 2);
        tile_save(2, 0);
        tile_save(2, 1);
        tile_save(2, 2);
    }

    /**
     * merge existing dtm for dynamic merge
     */
    void merge();
    void merge(cell_info_t& dst, const cell_info_t& src) const;

    /**
     * Save Z_MEAN as a normalized grayscale image in filepath
     */
    void export8u(const std::string& filepath) const {
        gdalwrap::gdal heightmap;
        heightmap.copy_meta_only(meta);
        heightmap.names = {"Z_MEAN"};
        heightmap.set_size(1, width, height);
        // get min/max from data
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();
        for (const auto& cell : internal) {
            if (cell[N_POINTS] > 0.9) {
                if (cell[Z_MEAN] > max)
                    max = cell[Z_MEAN];
                if (cell[Z_MEAN] < min)
                    min = cell[Z_MEAN];
            }
        }
        float diff = max - min;
        if (diff == 0) // max == min (useless band)
            return;

        float coef = 255.0 / diff;
        std::transform(internal.begin(), internal.end(),
            heightmap.bands[0].begin(),
            [&](const cell_info_t& cell) -> float {
                if (cell[N_POINTS] > 0.9)
                    return coef * (cell[Z_MEAN] - min);
                else
                    return -1;
            });
        heightmap.metadata["ATLAAS_MIN"] = std::to_string(min);
        heightmap.metadata["ATLAAS_MAX"] = std::to_string(max);
        heightmap.export8u(filepath, 0);
    }
    void export_zmean(const std::string& filepath) const {
        gdalwrap::gdal heightmap;
        heightmap.copy_meta_only(meta);
        heightmap.names = {"Z_MEAN"};
        heightmap.set_size(1, width, height);
        // TODO heightmap . set NoData = -10000
        std::transform(internal.begin(), internal.end(),
            heightmap.bands[0].begin(),
            [&](const cell_info_t& cell) -> float {
                return (cell[N_POINTS] > 0.9) ? cell[Z_MEAN] : -10000;
            });
        heightmap.save(filepath);
    }
};

} // namespace atlaas

#endif // ATLAAS_HPP

