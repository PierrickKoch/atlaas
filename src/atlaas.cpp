/*
 * atlaas.cpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-10-08
 * license: BSD
 */
#include <algorithm>        // for max
#include <stdexcept>        // for out_of_range

#include "atlaas/atlaas.hpp"

namespace atlaas {

void atlaas::merge(const points& cloud) {
    float z_mean, new_z;
    // merge point-cloud in internal structure
    for (const auto& point : cloud) {
        try {
            auto& info = internal[ map.index_custom(point[0], point[1]) ];
            new_z = point[3];
            // increment N_POINTS
            info[0]++;
            // update Z_MAX
            info[1] = std::max( info[1], static_cast<float>(new_z) );
            z_mean  = info[2];
            // compute Z_MEAN
            info[2] = info[2] * (info[0] - 1) + new_z / info[0];
            // update SIGMA_Z
            info[3] = ((info[0] - 1) * (info[3] + z_mean * z_mean) + new_z * new_z) /
                      info[0] - info[2] * info[2];
        } catch (std::out_of_range oor) {
            // point is outside the map
        }
    }
}

void atlaas::update() {
    // update map from internal
    // internal -> map
    for (size_t idx = 0; idx < internal.size(); idx++) {
        map.bands[0][idx] = internal[idx][0]; // N_POINTS
        map.bands[1][idx] = internal[idx][1]; // Z_MAX
        map.bands[2][idx] = internal[idx][2]; // Z_MEAN
        map.bands[3][idx] = internal[idx][3]; // SIGMA_Z
    }
}

void atlaas::_fill_internal() {
    // fill internal from map
    // map -> internal
    for (size_t idx = 0; idx < internal.size(); idx++) {
        internal[idx][0] = map.bands[0][idx]; // N_POINTS
        internal[idx][1] = map.bands[1][idx]; // Z_MAX
        internal[idx][2] = map.bands[2][idx]; // Z_MEAN
        internal[idx][3] = map.bands[3][idx]; // SIGMA_Z
    }
}

} // namespace atlaas
