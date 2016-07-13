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
#include <string>

#include <gdalwrap/gdal.hpp>
#include <atlaas/common.hpp>
#include <atlaas/region.hpp>
#include <atlaas/io.hpp>

namespace atlaas {

/**
 * atlaas
 */
class atlaas {
    /**
     * keep track of time at which we receive each cloud
     * to be able to correct them if needed in the future
     */
    std::vector<uint64_t> pcd_time;
    std::vector<map_id_t> pcd_map;
    std::vector<point_xy_t> pcd_xy;

    /**
     * I/O data model
     */
    gdalwrap::gdal meta;
    mutable gdalwrap::gdal tile;

    std::string atlaas_path;

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
    uint64_t time_base;

    /**
     * reprocess in progress
     */
    bool reprocess_in_progress;
    bool use_swap;

    /**
     * milliseconds since the base time.
     *
     * Since we'll store datas as float32, time since epoch would give
     * something like `1.39109e+09`, we substract a time_base
     * (to be set using `atlaas::set_time_base(time_t)`).
     */
    float get_reference_time() {
        return milliseconds_since_epoch() - time_base;
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
        meta.set_size(width, height); // does not change the container
        meta.set_transform(custom_x, custom_y, scale, -scale);
        meta.set_utm(utm_zone, utm_north);
        meta.set_custom_origin(custom_x, custom_y, custom_z);
        set_time_base( milliseconds_since_epoch() );
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
        atlaas_path = getenv("ATLAAS_PATH", ".");
        reprocess_in_progress = false;
        use_swap = true;
    }

    void set_use_swap(bool value) {
        use_swap = value;
    }

    std::string get_atlaas_path() {
        return atlaas_path;
    }

    void set_atlaas_path(const std::string& path) {
        atlaas_path = path;
    }

    std::string tilepath(int x, int y) const {
        std::ostringstream oss;
        oss << atlaas_path << "/atlaas." << x << "x" << y << ".tif";
        return oss.str();
    }

    std::string tilepath(const map_id_t& p) const {
        return tilepath(p[0], p[1]);
    }

    void set_time_base(uint64_t base) {
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
    void merge(points& cloud, const matrix& transformation,
        const covmat& covariance, bool dump = true);
    void merge(points& cloud, const matrix& transformation, bool dump = true) {
        merge(cloud, transformation, {}, dump);
    }

    std::string cloud_filepath(size_t seq) const {
        std::ostringstream oss;
        oss << atlaas_path << "/" << cloud_filename(seq);
        return oss.str();
    }

    /**
     * write pcd file
     */
    void save_inc(const points& cloud, const matrix& transformation) {
        pcd_time.push_back(milliseconds_since_epoch());
        pcd_map.push_back(current);
        pcd_xy.push_back(matrix_to_point(transformation));
        save(cloud_filepath( pcd_time.size() - 1 ), cloud, transformation);
    }

    /**
     * merge from raw C array (using std::copy !)
     * used for numpy -> C++ interface
     * transformation must be double[16] : row-major Matrix(4,4)
     * cloud must be float[cloud_len1][cloud_len2]
     * cloud_len2 must be either 3 (XYZ) or 4 (XYZI)
     */
    void c_merge(const float* cloud, size_t cloud_len1, size_t cloud_len2,
                 const double* transformation, const double* covariance,
                 bool dump = true) {
        matrix tr;
        covmat cv;
        points cd( cloud_len1 );
        std::copy(covariance, covariance + 36, cv.begin());
        std::copy(transformation, transformation + 16, tr.begin());
        for (size_t i = 0; i < cloud_len1; i++)
            std::copy(cloud+i*cloud_len2, cloud+(i+1)*cloud_len2, cd[i].begin());
        merge(cd, tr, cv, dump);
    }

    size_t process(size_t start = 0, size_t end = std::numeric_limits<size_t>::max()) {
        points cloud;
        matrix transformation = {{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 }};
        std::string filepath;
        for (; start <= end; start++ ) {
            filepath = cloud_filepath(start);
            if ( ! file_exists(filepath) )
                break;
            load(filepath, cloud, transformation);
            merge(cloud, transformation, {}, false);
        }
        return start;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Magic fix path between 2 good GPS signals
    ////////////////////////////////////////////////////////////////////////////

    /**
     * 1.
     */
    size_t get_closest_pcd_id(uint64_t milliseconds) const {
        size_t i = 0;
        for (; i < pcd_time.size(); i++) {
            if (pcd_time[i] >= milliseconds) {
                if ((i > 0) && ((milliseconds - pcd_time[i-1]) <
                                (pcd_time[i] - milliseconds)))
                    i--;
                break;
            }
        }
        return i;
    }
    size_t get_closest_pcd_id_pose(point_xy_t pose, uint64_t tmax) const {
        size_t i = 0, best = 0;
        double dcur, dmin = std::numeric_limits<double>::max();
        for (; i < pcd_xy.size(); i++) {
            if (pcd_time[i] >= tmax)
                break;
            dcur = distance_sq(pose, pcd_xy[i]);
            if (dcur < dmin) {
                dmin = dcur;
                best = i;
            }
        }
        return best;
    }
    size_t c_closest_pcd(double x, double y, uint64_t tmax) const {
        return get_closest_pcd_id_pose({{x,y}}, tmax);
    }

    std::vector<size_t> get_pcd_same_map(size_t id) {
        std::vector<size_t> pcd_same_map;
        for (size_t inc = 0; inc < pcd_map.size(); inc++) {
            if (inc != id and
                pcd_map[inc][0] == pcd_map[id][0] and
                pcd_map[inc][1] == pcd_map[id][1]) {
                pcd_same_map.push_back(inc);
            }
        }
        return pcd_same_map;
    }
    std::map<size_t, float> get_pcd_overlap(size_t id) {
        std::map<size_t, float> overlap_score;
        std::vector<size_t> pcd_same_map = get_pcd_same_map(id);
        cells_info_t c_ref( width * height ); // reference
        cells_info_t c_cmp( width * height ); // to compare
        points cloud;
        matrix transformation;
        load(cloud_filename(id), cloud, transformation);
        transform(cloud, transformation);
        rasterize(cloud, c_ref); // XXX assume meta is in the map ref id
        for (size_t pcd_id : pcd_same_map) {
            load(cloud_filename(pcd_id), cloud, transformation);
            transform(cloud, transformation);
            // clear the dynamic map (zeros)
            cell_info_t zeros{}; // value-initialization w/empty initializer
            std::fill(c_cmp.begin(), c_cmp.end(), zeros);
            rasterize(cloud, c_cmp); // XXX assume meta is in the map ref id
            // get overlap between c_ref and c_cmp
            float overlap = 0;
            for (size_t i = 0; i < c_ref.size(); i++) {
                if (c_ref[i][N_POINTS] > 0 and c_cmp[i][N_POINTS] > 0)
                    overlap += 1;
            }
            overlap /= c_ref.size();
            overlap_score[pcd_id] = overlap;
        }
        return overlap_score;
    }
    std::string get_pcd_overlap_str(size_t id) {
        return to_string(get_pcd_overlap(id));
    }


    void clear_all() {
        // clear the dynamic map (zeros)
        cell_info_t zeros{}; // value-initialization w/empty initializer
        std::fill(dyninter.begin(), dyninter.end(), zeros);
        std::fill(gndinter.begin(), gndinter.end(), zeros);
        std::fill(internal.begin(), internal.end(), zeros);
    }

    size_t reprocess2(std::vector<matrix> alltr, size_t start = 0) {
        reprocess_in_progress = true; // "lock" merge, do not fuse anymore
        size_t nscans = alltr.size();
        size_t lscans = start+nscans;
        assert(pcd_time.size() >= lscans);
        points cloud;
        matrix transformation;
        // 4. overwrite correct pcd pose
        for (size_t i = start; i < lscans; i++) {
            std::string filepath = cloud_filepath( i );
            load(filepath, cloud, transformation);
            save(filepath, cloud, alltr[i-start]);
        }
        // 5. backup all atlaas.*.tif from map_id related to the fix
        for (size_t i = start; i < lscans; i++) {
            for (uint sx=0; sx <= 2; sx++)
            for (uint sy=0; sy <= 2; sy++) {
                std::string tile_path = tilepath(pcd_map[i][0]+sx,
                                                 pcd_map[i][1]+sy);
                if ( file_exists(tile_path) ) {
                    std::ostringstream oss;
                    oss << tile_path << "." << nscans << ".bak";
                    std::rename( tile_path.c_str() , oss.str().c_str() );
                }
            }
        }
        // 6. clear internal
        clear_all();
        // 7. process all pcds from start to last
        //    (even the ones after reprocess call)
        size_t result = process(start);
        reprocess_in_progress = false;
        assert(result >= nscans);
        return result;
    }

    /**
     * last_good_pose:  milliseconds since epoch
     * time_of_fix:     milliseconds since epoch
     * point_xy_t:      2D point {x,y}
     */
    size_t reprocess(uint64_t last_good_pose, uint64_t time_of_fix, double fixed_pose_x, double fixed_pose_y) {
        // 1. find the pcd id at time `last_good_pose`
        size_t last_good_pcd_id = get_closest_pcd_id(last_good_pose),
               fixed_pcd_id = get_closest_pcd_id(time_of_fix);
        // 2. build correct path
        if (last_good_pcd_id >= fixed_pcd_id)
            return 0;
        points cloud;
        matrix transformation;
        std::vector<point_xy_t> path(1+fixed_pcd_id-last_good_pcd_id);
        for (size_t i = last_good_pcd_id, j = 0; i <= fixed_pcd_id; i++, j++) {
            load(cloud_filepath( i ), cloud, transformation);
            path[j] = matrix_to_point(transformation) ;
        }
        // 3. get path length
        float dist = 0;
        std::vector<float> inc_dist(path.size());
        inc_dist[0] = 0; // first good pose no diff
        for (size_t i = 1; i < path.size(); i++) {
            dist += distance(path[i-1], path[i]);
            inc_dist[i] = dist;
        }
        // 4. overwrite correct pcd pose
        double dx = fixed_pose_x - path[path.size()-1][0],
               dy = fixed_pose_y - path[path.size()-1][1];
        for (size_t i = last_good_pcd_id + 1, j = 0; i <= fixed_pcd_id; i++) {
            std::string filepath = cloud_filepath( i );
            load(filepath, cloud, transformation);
            float factor = inc_dist[j++] / dist;
            transformation[3] += dx * factor; // transformation.x
            transformation[7] += dy * factor; // transformation.y
            save(filepath, cloud, transformation);
        }
        // 5. backup all atlaas.*.tif from map_id related to the fix
        for (size_t i = last_good_pcd_id + 1; i <= fixed_pcd_id; i++) {
            for (uint sx=0; sx <= 2; sx++)
            for (uint sy=0; sy <= 2; sy++) {
                std::string tile_path = tilepath(pcd_map[i][0]+sx,
                                                 pcd_map[i][1]+sy);
                if ( file_exists(tile_path) ) {
                    std::ostringstream oss;
                    oss << tile_path << "." << fixed_pcd_id << ".bak";
                    std::rename( tile_path.c_str() , oss.str().c_str() );
                }
            }
        }
        // 6. clear internal
        clear_all();
        // 7. process all pcds from the last good pcd id
        return process(last_good_pcd_id) - last_good_pcd_id;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    /**
     * Load a cloud and a transformation from file for replay
     */
    void merge(const std::string& filepath, bool dump = false) {
        points cloud;
        matrix transformation;
        load(filepath, cloud, transformation);
        merge(cloud, transformation, {}, dump);
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

    void region(const std::string& file_out) {
        save_currents();
        glob_region(atlaas_path + "/atlaas.*x*.tif", file_out);
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
    std::string pcd_xy_str() const {
        return to_string(pcd_xy);
    }
    std::string pcd_map_str() const {
        return to_string(pcd_map);
    }
    std::string pcd_time_str() const {
        return to_string(pcd_time);
    }
};

} // namespace atlaas

#endif // ATLAAS_HPP
