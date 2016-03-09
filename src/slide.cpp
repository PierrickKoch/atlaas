/*
 * slide.cpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2014-03-19
 * license: BSD
 */
#include <algorithm>        // std::copy{,_backward}

#include <atlaas/atlaas.hpp>

namespace atlaas {

/**
 * Slide, save, load tiles
 *
 * @returns whether we did slide or not,
 *          useful to check if we need multiple slide at init
 */
bool atlaas::slide() {
    const point_xy_t& pixr = meta.point_custom2pix(sensor_xy[0], sensor_xy[1]);
    float cx = pixr[0] / width;
    float cy = pixr[1] / height;
    // check, slide, save, load
    if ( ( cx > 0.25 ) && ( cx < 0.75 ) &&
         ( cy > 0.25 ) && ( cy < 0.75 ) )
        return false; // robot is in "center" square

    int dx = (cx < 0.33) ? -1 : (cx > 0.66) ? 1 : 0; // W/E
    int dy = (cy < 0.33) ? -1 : (cy > 0.66) ? 1 : 0; // N/S
    cell_info_t zeros{}; // value-initialization w/empty initializer
    // reset state and ground infos used for dynamic merge
    std::fill(gndinter.begin(), gndinter.end(), zeros);

    if (dx == -1) {
        // save EAST 1/3 maplets [ 1,-1], [ 1, 0], [ 1, 1]
        tile_save(2, 0);
        tile_save(2, 1);
        tile_save(2, 2);
        if (dy == -1) {
            // save SOUTH
            tile_save(0, 2);
            tile_save(1, 2);
        } else if (dy == 1) {
            // save NORTH
            tile_save(0, 0);
            tile_save(1, 0);
        }
        // move the map to the WEST [-1 -> 0; 0 -> 1]
        for (auto it = internal.begin(); it < internal.end(); it += width) {
            std::copy_backward(it, it + 2 * sw, it + width);
            // reset(it, it + sw);
            std::fill(it, it + sw, zeros);
        }
    } else if (dx == 1) {
        // save WEST 1/3 maplets [-1,-1], [-1, 0], [-1, 1]
        tile_save(0, 0);
        tile_save(0, 1);
        tile_save(0, 2);
        if (dy == -1) {
            // save SOUTH
            tile_save(1, 2);
            tile_save(2, 2);
        } else if (dy == 1) {
            // save NORTH
            tile_save(1, 0);
            tile_save(2, 0);
        }
        // move the map to the EAST
        for (auto it = internal.begin(); it < internal.end(); it += width) {
            std::copy(it + sw, it + width, it);
            // reset(it + 2 * sw, it + width);
            std::fill(it + 2 * sw, it + width, zeros);
        }
    } else if (dy == -1) {
        // save SOUTH
        tile_save(0, 2);
        tile_save(1, 2);
        tile_save(2, 2);
    } else if (dy == 1) {
        // save NORTH
        tile_save(0, 0);
        tile_save(1, 0);
        tile_save(2, 0);
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
        tile_load(0, 0);
        tile_load(0, 1);
        tile_load(0, 2);
        if (dy == -1) {
            // load NORTH
            tile_load(1, 0);
            tile_load(2, 0);
        } else if (dy == 1) {
            // load SOUTH
            tile_load(1, 2);
            tile_load(2, 2);
        }
    } else if (dx == 1) {
        // load EAST maplets
        tile_load(2, 0);
        tile_load(2, 1);
        tile_load(2, 2);
        if (dy == -1) {
            // load NORTH
            tile_load(0, 0);
            tile_load(1, 0);
        } else if (dy == 1) {
            // load SOUTH
            tile_load(0, 2);
            tile_load(1, 2);
        }
    } else if (dy == -1) {
        // load NORTH
        tile_load(0, 0);
        tile_load(1, 0);
        tile_load(2, 0);
    } else if (dy == 1) {
        // load SOUTH
        tile_load(0, 2);
        tile_load(1, 2);
        tile_load(2, 2);
    }

    const auto& utm = meta.point_pix2utm(sw * dx, sh * dy);
    // update map transform used for merging the pointcloud
    meta.set_transform(utm[0], utm[1], meta.get_scale_x(), meta.get_scale_y());
    LOG( __func__ << " utm " << utm[0] << ", " << utm[1] );

    return true;
}

} // namespace atlaas
