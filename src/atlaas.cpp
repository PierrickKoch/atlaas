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

void atlaas::merge(const points& cloud) {
    float z_mean, n_pts;
    double new_z;
    // merge point-cloud in internal structure
    for (const auto& point : cloud) {
        try {
            auto& info = internal[ map.index_custom(point[0], point[1]) ];
            new_z = point[3];
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
        } catch (std::out_of_range oor) {
            // point is outside the map
        }
    }
}

void atlaas::merge(const gdalwrap::gdal& gmap) {
    assert( map.names == MAP_NAMES );
    // merge existing dtm
    size_t idx = 0;
    double scale_x = gmap.get_scale_x();
    double scale_y = gmap.get_scale_y();
    point_xy_t utm = gmap.point_pix2utm(0, 0);
    double x_origin = utm[0];
    float z_mean, n_pts, gmap_n_pts;

    for (size_t ix = 0; ix < gmap.get_width();  ix++) {
        for (size_t iy = 0; iy < gmap.get_height(); iy++) {
            utm[0] += scale_x;
            idx += 1;
            try {
                // XXX index_utm() must take care of orientation (near future)
                auto& info = internal[ map.index_utm(utm[0], utm[1]) ];
                n_pts = info[N_POINTS];
                z_mean = info[Z_MEAN];
                gmap_n_pts = gmap.bands[N_POINTS][idx];
                info[N_POINTS] += gmap_n_pts;
                if (info[Z_MAX] < gmap.bands[Z_MAX][idx])
                    info[Z_MAX] = gmap.bands[Z_MAX][idx];
                if (info[Z_MIN] > gmap.bands[Z_MIN][idx])
                    info[Z_MIN] = gmap.bands[Z_MIN][idx];
                info[Z_MEAN] = ( (z_mean * n_pts) + (gmap.bands[Z_MEAN][idx] *
                    gmap_n_pts) ) / info[N_POINTS];
                // TODO SIGMA_Z
            } catch (std::out_of_range oor) {
                // point is outside the map
            }
        }
        utm[0] = x_origin;
        utm[1] += scale_y;
    }
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
}

} // namespace atlaas
