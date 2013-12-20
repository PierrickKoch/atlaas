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

#include <fstream>          // ofstream, tmplog
#include <algorithm>        // copy{,_backward}
#include <cmath>            // floor

#include "atlaas/atlaas.hpp"

namespace atlaas {

static std::ofstream tmplog("atlaas.log");

/**
 * Apply the transformation matrix to the point cloud (in place)
 */
void transform(points& cloud, const matrix& tr) {
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
 * Merge point cloud in the internal model
 * with the sensor to world transformation,
 * and slide, save, load submodels.
 *
 * @param cloud: point cloud in the sensor frame
 * @param transformation: sensor to world transformation
 */
void atlaas::merge(points& cloud, const matrix& transformation) {
    transform(cloud, transformation);
    // according to t3d/src/matrix.c:t3dMatrixOfEuler : [3,7] = x,y
    slide_to(transformation[3], transformation[7]);
    merge(cloud);
}

void atlaas::sub_load(int sx, int sy) {
    std::string filepath = sub_name(current[0] + sx, current[1] + sy);
    if ( ! file_exists( filepath ) )
        return; // no file to load
    sub->init(filepath);
    auto it  = internal.begin() + sw * (sx + 1) + sh * width * (sy + 1),
         end = it + sh * width;
    for (auto sit = sub->internal.begin(); it < end; it += width, sit += sw) {
        // sub to map
        std::copy(sit, sit + sw, it);
    }
    map_sync = false;
}

void atlaas::sub_save(int sx, int sy) const {
    auto it  = internal.begin() + sw * (sx + 1) + sh * width * (sy + 1),
         end = it + sh * width;
    for (auto sit = sub->internal.begin(); it < end; it += width, sit += sw) {
        // map to sub
        std::copy(it, it + sw, sit);
    }
    sub->update();
    const auto& utm = map.point_pix2utm( sx * sw, sy * sh);
    // update map transform used for merging the pointcloud
    sub->map.set_transform(utm[0], utm[1], map.get_scale_x(), map.get_scale_y());
    sub->map.save( sub_name(current[0] + sx, current[1] + sy) );
}

/**
 * Slide, save, load submodels
 *
 * @param robx:  robot x pose in the custom frame
 * @param roby:  robot y pose in the custom frame
 */
void atlaas::slide_to(double robx, double roby) {
    const point_xy_t& pixr = map.point_custom2pix(robx, roby);
    float cx = pixr[0] / width;
    float cy = pixr[1] / height;
    // check, slide, save, load
    if ( ( cx > 0.25 ) && ( cx < 0.75 ) &&
         ( cy > 0.25 ) && ( cy < 0.75 ) )
        return; // robot is in "center" square

    int dx = (cx < 0.33) ? -1 : (cx > 0.66) ? 1 : 0; // W/E
    int dy = (cy < 0.33) ? -1 : (cy > 0.66) ? 1 : 0; // N/S
    point_info_t zeros{}; // value-initialization w/empty initializer

    if (dx == -1) {
        // save EAST 1/3 maplets [ 1,-1], [ 1, 0], [ 1, 1]
        sub_save( 1, -1);
        sub_save( 1,  0);
        sub_save( 1,  1);
        if (dy == -1) {
            // save SOUTH
            sub_save(-1,  1);
            sub_save( 0,  1);
        } else if (dy == 1) {
            // save NORTH
            sub_save(-1, -1);
            sub_save( 0, -1);
        }
        // move the map to the WEST [-1 -> 0; 0 -> 1]
        for (auto it = internal.begin(); it < internal.end(); it += width) {
            std::copy_backward(it, it + 2 * sw, it + width);
            // reset(it, it + sw);
            std::fill(it, it + sw, zeros);
        }
    } else if (dx == 1) {
        // save WEST 1/3 maplets [-1,-1], [-1, 0], [-1, 1]
        sub_save(-1, -1);
        sub_save(-1,  0);
        sub_save(-1,  1);
        if (dy == -1) {
            // save SOUTH
            sub_save( 0,  1);
            sub_save( 1,  1);
        } else if (dy == 1) {
            // save NORTH
            sub_save( 0, -1);
            sub_save( 1, -1);
        }
        // move the map to the EAST
        for (auto it = internal.begin(); it < internal.end(); it += width) {
            std::copy(it + sw, it + width, it);
            // reset(it + 2 * sw, it + width);
            std::fill(it + 2 * sw, it + width, zeros);
        }
    } else if (dy == -1) {
        // save SOUTH
        sub_save(-1,  1);
        sub_save( 0,  1);
        sub_save( 1,  1);
    } else if (dy == 1) {
        // save NORTH
        sub_save(-1, -1);
        sub_save( 0, -1);
        sub_save( 1, -1);
    }

    if (dy == -1) {
        std::copy_backward(internal.begin(), internal.end() - sh * width,
                           internal.end());
        // reset(internal.begin(), internal.begin() + sh * width);
        std::fill(internal.begin(), internal.begin() + sh * width - 1, zeros);
    } else if (dy == 1) {
        std::copy(internal.begin() + sh * width, internal.end(), internal.begin());
        // reset(internal.end() - sh * width, internal.end());
        std::fill(internal.end() - sh * width, internal.end(), zeros);
    }

    // after moving, update our current center
    current[0] += dx;
    current[1] += dy;

    // load here
    if (dx == -1) {
        // load WEST maplets
        sub_load(-1, -1);
        sub_load(-1,  0);
        sub_load(-1,  1);
        if (dy == -1) {
            // load NORTH
            sub_load( 0, -1);
            sub_load( 1, -1);
        } else if (dy == 1) {
            // load SOUTH
            sub_load( 0,  1);
            sub_load( 1,  1);
        }
    } else if (dx == 1) {
        // load EAST maplets
        sub_load( 1, -1);
        sub_load( 1,  0);
        sub_load( 1,  1);
        if (dy == -1) {
            // load NORTH
            sub_load(-1, -1);
            sub_load( 0, -1);
        } else if (dy == 1) {
            // load SOUTH
            sub_load(-1,  1);
            sub_load( 0,  1);
        }
    } else if (dy == -1) {
        // load NORTH
        sub_load(-1, -1);
        sub_load( 0, -1);
        sub_load( 1, -1);
    } else if (dy == 1) {
        // load SOUTH
        sub_load(-1,  1);
        sub_load( 0,  1);
        sub_load( 1,  1);
    }

    const auto& utm = map.point_pix2utm(sw * dx, sh * dy);
    // update map transform used for merging the pointcloud
    map.set_transform(utm[0], utm[1], map.get_scale_x(), map.get_scale_y());
    map_sync = false;
    tmplog << __func__ << " utm " << utm[0] << ", " << utm[1] << std::endl;
}

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

        if (n_pts < 1) {
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

    for (size_t ix = 0; ix < from.width;  ix++) {
        for (size_t iy = 0; iy < from.height; iy++) {
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
    width  = map.get_width();  // x
    height = map.get_height(); // y
    sw = width  / 3; // sub-width
    sh = height / 3; // sub-height
    // set internal size
    internal.resize( width * height );
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
