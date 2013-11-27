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
#include <stdexcept>        // for out_of_range

#include "atlaas/atlaas.hpp"

namespace atlaas {

/**
 * Merge a point cloud in the internal model
 *
 * @param cloud: point cloud in the custom frame
 */
void atlaas::merge(const points& cloud) {
    size_t index;
    float z_mean, n_pts, new_z;
    // merge point-cloud in internal structure
    for (const auto& point : cloud) {
        index = map.index_custom(point[0], point[1]);
        if (index == std::numeric_limits<size_t>::max() )
            continue; // point is outside the map

        auto& info = internal[ index ];
        new_z = point[2];
        n_pts = info[N_POINTS];
        z_mean = info[Z_MEAN];
        // increment N_POINTS
        info[N_POINTS]++;

        if (n_pts == 0) {
            info[Z_MAX] = new_z;
            info[Z_MIN] = new_z;
        } else {
            // update Z_MAX
            if (new_z > info[Z_MAX])
                info[Z_MAX] = new_z;
            // update Z_MIN
            if (new_z < info[Z_MIN])
                info[Z_MIN] = new_z;
        }
        /* Incremental mean and variance updates (according to Knuth's bible,
           Vol. 2, section 4.2.2). The actual variance will later be divided
           by the number of samples plus 1. */
        info[Z_MEAN]   = (z_mean * n_pts + new_z) / info[N_POINTS];
        info[SIGMA_Z] += (new_z - z_mean) * (new_z - info[Z_MEAN]);

    }
    map_sync = false;
}

void atlaas::merge(const atlaas& from) {
    assert( map.names == MAP_NAMES );
    // merge existing dtm
    size_t index, idx = 0;
    double scale_x = from.map.get_scale_x();
    double scale_y = from.map.get_scale_y();
    point_xy_t utm = from.map.point_pix2utm(0, 0);
    double x_origin = utm[0];
    float z_mean, d_mean, n_pts, from_n_pts;

    for (size_t ix = 0; ix < from.map.get_width();  ix++) {
        for (size_t iy = 0; iy < from.map.get_height(); iy++) {
            utm[0] += scale_x;
            idx += 1;

            // XXX index_utm() must take care of orientation (near future)
            index = map.index_utm(utm[0], utm[1]);
            if (index == std::numeric_limits<size_t>::max() ) {
                // TODO point is outside the map
                continue;
            }
            auto& info = internal[ index ];
            const auto& info_from = from.internal[idx];
            n_pts = info[N_POINTS];
            z_mean = info[Z_MEAN];
            from_n_pts = info_from[N_POINTS];
            info[N_POINTS] += from_n_pts;
            if (info[Z_MAX] < info_from[Z_MAX])
                info[Z_MAX] = info_from[Z_MAX];
            if (info[Z_MIN] > info_from[Z_MIN])
                info[Z_MIN] = info_from[Z_MIN];
            info[Z_MEAN] = ( (z_mean * n_pts) + (info_from[Z_MEAN] *
                from_n_pts) ) / info[N_POINTS];
            /* The actual variance will later be divided by the number of
               samples plus 1. */
            d_mean = info_from[Z_MEAN] - z_mean;
            info[SIGMA_Z] = info[SIGMA_Z] * info[SIGMA_Z] * n_pts +
                info_from[SIGMA_Z] * info_from[SIGMA_Z] * from_n_pts +
                d_mean * d_mean * n_pts * from_n_pts / info[N_POINTS];
        }
        utm[0] = x_origin;
        utm[1] += scale_y;
    }
    map_sync = false;
}

void atlaas::update() {
    // update map from internal
    // internal -> map
    for (size_t idx = 0; idx < internal.size(); idx++) {
        map.bands[N_POINTS][idx]    = internal[idx][N_POINTS];
        map.bands[Z_MAX][idx]       = internal[idx][Z_MAX];
        map.bands[Z_MIN][idx]       = internal[idx][Z_MIN];
        map.bands[Z_MEAN][idx]      = internal[idx][Z_MEAN];
        map.bands[SIGMA_Z][idx]     = internal[idx][SIGMA_Z];
    }
    map_sync = true;
}

void atlaas::_fill_internal() {
    assert( map.names == MAP_NAMES );
    // set internal size
    internal.resize( map.get_width() * map.get_height() );
    // fill internal from map
    // map -> internal
    for (size_t idx = 0; idx < internal.size(); idx++) {
        internal[idx][N_POINTS]     = map.bands[N_POINTS][idx];
        internal[idx][Z_MAX]        = map.bands[Z_MAX][idx];
        internal[idx][Z_MIN]        = map.bands[Z_MIN][idx];
        internal[idx][Z_MEAN]       = map.bands[Z_MEAN][idx];
        internal[idx][SIGMA_Z]      = map.bands[SIGMA_Z][idx];
    }
    map_sync = true;
}

} // namespace atlaas
