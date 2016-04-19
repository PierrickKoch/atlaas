/*
 * atlaas.cpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-10-08
 * license: BSD
 */
#include <cassert>

#include <atlaas/atlaas.hpp>
#include <atlaas/common.hpp>

namespace atlaas {

/**
 * Merge point cloud in the internal model
 * with the sensor to world transformation,
 * and slide, save, load tiles.
 *
 * @param cloud: point cloud in the sensor frame
 * @param transformation: sensor to world transformation
 */
void atlaas::merge(points& cloud, const matrix& transformation,
        const covmat& covariance, bool dump) {
    if (cloud.size() < 1)
        return; // pcl writeBinaryCompressed crash with empty cloud
    if (dump) {
        save_inc(cloud, transformation);
        if (reprocess_in_progress)
            return;
    }

    sensor_xy = matrix_to_point(transformation);
    // slide map while needed
    do_slide();
    // use dynamic merge
    // clear the dynamic map (zeros)
    cell_info_t zeros{}; // value-initialization w/empty initializer
    std::fill(dyninter.begin(), dyninter.end(), zeros);
    // transform the cloud from sensor to custom frame
    transform(cloud, transformation);
    // merge the point-cloud
    rasterize(cloud, dyninter);

    // merge the dynamic atlaas with internal data
    merge();
}

void atlaas::tile_load(int sx, int sy) {
    std::string filepath = tilepath(current[0] + sx, current[1] + sy);
    if ( ! file_exists( filepath ) )
        return; // no file to load
    tile.load(filepath);
    assert( tile.bands.size() == MAP_NAMES.size() );
    assert( tile.bands[0].size() == sw * sh );
    // update each cell time if bases differ
    long diff = time_base - std::stol(tile.get_meta("TIME", "0"));
    size_t idx = 0, eoi = 0;
    for (auto it  = internal.begin() + sw * sx + sh * width * sy,
         end = it + sh * width; it < end; it += width - sw)
    for (eoi += sw; idx < eoi; idx++) {
        // tile to map
        (*it)[N_POINTS] = tile.bands[N_POINTS][idx];
        (*it)[Z_MAX]    = tile.bands[Z_MAX][idx];
        (*it)[Z_MIN]    = tile.bands[Z_MIN][idx];
        (*it)[Z_MEAN]   = tile.bands[Z_MEAN][idx];
        (*it)[VARIANCE] = tile.bands[VARIANCE][idx];
        (*it)[TIME]     = tile.bands[TIME][idx];
        (*it)[DIST_SQ]  = tile.bands[DIST_SQ][idx];
        if ( diff and (*it)[N_POINTS] > 0.9 )
            (*it)[TIME] -= diff;
        it++;
    }
}

void atlaas::tile_save(int sx, int sy) const {
    // re-init the IO cache in case a load corrupted its meta-data
    // reset meta-data (important for TIME)
    tile.copy_meta_only(meta);
    tile.names = MAP_NAMES;
    tile.set_size(N_RASTER, sw, sh);
    size_t idx = 0, eoi = 0;
    for (auto it  = internal.begin() + sw * sx + sh * width * sy,
         end = it + sh * width; it < end; it += width - sw)
    for (eoi += sw; idx < eoi; idx++) {
        // map to tile
        tile.bands[N_POINTS][idx] = (*it)[N_POINTS];
        tile.bands[Z_MAX][idx]    = (*it)[Z_MAX];
        tile.bands[Z_MIN][idx]    = (*it)[Z_MIN];
        tile.bands[Z_MEAN][idx]   = (*it)[Z_MEAN];
        tile.bands[VARIANCE][idx] = (*it)[VARIANCE];
        tile.bands[TIME][idx]     = (*it)[TIME];
        tile.bands[DIST_SQ][idx]  = (*it)[DIST_SQ];
        it++;
    }
    if (any_gt_zero(tile.bands[N_POINTS])) { // dont save empty tiles
        const auto& utm = meta.point_pix2utm( sx * sw, sy * sh);
        // update map transform used for merging the pointcloud
        tile.set_transform(utm[0], utm[1], meta.get_scale_x(), meta.get_scale_y());
        tile.save( tilepath(current[0] + sx, current[1] + sy) );
    }
}

/**
 * Merge a point cloud in the internal model
 *
 * @param cloud: point cloud in the custom frame
 * @param inter: an internal container of cells
 */
void atlaas::rasterize(const points& cloud, cells_info_t& inter) const {
    size_t index;
    float z_mean, n_pts, new_z;
    // merge point-cloud in internal structure
    for (const auto& point : cloud) {
        index = meta.index_custom(point[0], point[1]);
        if (index >= inter.size() )
            continue; // point is outside the map

        auto& info = inter[ index ];
        new_z = point[2];
        n_pts = info[N_POINTS];

        if (n_pts < 1) {
            info[N_POINTS] = 1;
            info[Z_MAX]  = new_z;
            info[Z_MIN]  = new_z;
            info[Z_MEAN] = new_z;
            info[VARIANCE] = 0;
            info[DIST_SQ] = distance_sq(sensor_xy, {{point[0], point[1]}});
        } else {
            z_mean = info[Z_MEAN];
            // increment N_POINTS
            info[N_POINTS]++;
            // update Z_MAX
            if (new_z > info[Z_MAX])
                info[Z_MAX] = new_z;
            // update Z_MIN
            if (new_z < info[Z_MIN])
                info[Z_MIN] = new_z;

            /* Incremental mean and variance updates (according to Knuth's bible,
               Vol. 2, section 4.2.2). The actual variance will later be divided
               by the number of samples plus 1. */
            info[Z_MEAN]    = (z_mean * n_pts + new_z) / info[N_POINTS];
            info[VARIANCE] += (new_z - z_mean) * (new_z - info[Z_MEAN]);
        }
    }
}

/**
 * Merge dynamic dtm
 */
void atlaas::merge() {
    bool is_vertical;
    size_t index = 0;
    float time_ref = get_reference_time();
    auto it = internal.begin();

    for (auto& dyninfo : dyninter) {
        if ( dyninfo[N_POINTS] > 0 && (
            (*it)[N_POINTS] < 1 || (*it)[DIST_SQ] - dyninfo[DIST_SQ] > -4 ) ) {
            /* compute the real variance (according to Knuth's bible) */
            if (dyninfo[N_POINTS] > 2)
                dyninfo[VARIANCE] /= dyninfo[N_POINTS] - 1;

            is_vertical = dyninfo[VARIANCE] > variance_threshold;

            if ( (*it)[N_POINTS] < 1 || ( dyninfo[N_POINTS] > 2 &&
                    (*it)[DIST_SQ] - dyninfo[DIST_SQ] > 4 ) ) {
                // init
                *it = dyninfo;
            } else if (use_swap) {
                if ( is_vertical == ( (*it)[VARIANCE] > variance_threshold) ) {
                    // same state
                    // if the cells are flat and differ more than 10cm, swap
                    if (!is_vertical && (( (*it)[Z_MEAN] - dyninfo[Z_MEAN] ) > 0.1 )) {
                        gndinter[index] = *it;
                        *it = dyninfo;
                        // TODO (*it)[DYNAMIC] += 1.0;
                    } else {
                        merge(*it, dyninfo);
                    }
                } else if ( is_vertical ) {
                    // was flat, backup the cell in ground swap
                    gndinter[index] = *it;
                    *it = dyninfo;
                    // TODO (*it)[DYNAMIC] += 1.0;
                } else {
                    // was vertical, revert ground and merge
                    *it = gndinter[index];
                    merge(*it, dyninfo);
                    // TODO (*it)[DYNAMIC] += 1.0;
                    // TODO gndinter[index] = zeros; ???
                }
            } else {
                merge(*it, dyninfo);
            }
            (*it)[TIME] = time_ref;
        }

        it++;
        index++;
    }
}

/**
 * Merge the two cells src and dst into dst
 */
void atlaas::merge(cell_info_t& dst, const cell_info_t& src) const {
    if ( dst[N_POINTS] < 1 ) {
        dst = src;
        return;
    }
    float z_mean, new_n_pts;

    new_n_pts = src[N_POINTS] + dst[N_POINTS];
    z_mean = dst[Z_MEAN];

    if (dst[Z_MAX] < src[Z_MAX])
        dst[Z_MAX] = src[Z_MAX];
    if (dst[Z_MIN] > src[Z_MIN])
        dst[Z_MIN] = src[Z_MIN];

    dst[Z_MEAN] = ( (z_mean * dst[N_POINTS]) + (src[Z_MEAN] * src[N_POINTS]) )
                   / new_n_pts;
    // compute the global variance
    dst[VARIANCE] = ( src[VARIANCE] * (src[N_POINTS] - 1)
                    + dst[VARIANCE] * (dst[N_POINTS] - 1)
                    ) / (new_n_pts - 1);
    dst[N_POINTS] = new_n_pts;
}

} // namespace atlaas
