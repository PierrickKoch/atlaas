/*
 * region.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2015-02-23
 * license: BSD
 */
#ifndef ATLAAS_REGION_HPP
#define ATLAAS_REGION_HPP

#include <glob.h>
#include <cmath>
#include <vector>
#include <string>
#include <atlaas/common.hpp>
#include <gdalwrap/gdal.hpp>

namespace atlaas {

static const float VARIANCE_THRESHOLD = 0.01;
static const float EDGE_FACTOR = 100;
static const float NO_DATA = -10000;

inline std::vector<std::string> glob(const std::string& pattern) {
    glob_t glob_result;
    std::vector<std::string> match;
    // might add |GLOB_BRACE for "{0-9}*x{0-9}*"
    // and |GLOB_TILDE for ~/path/from/home
    glob(pattern.c_str(), GLOB_NOSORT, NULL, &glob_result);
    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        match.push_back(glob_result.gl_pathv[i]);
    }
    globfree(&glob_result);
    return match;
}

inline void select(std::vector<gdalwrap::gdal>& files, float vt = VARIANCE_THRESHOLD) {
    for (gdalwrap::gdal& file : files) {
        for (size_t id = 0; id < file.bands[0].size(); id++) {
            if (file.bands[atlaas::N_POINTS][id] < 1) {
                file.bands[atlaas::Z_MEAN][id] = NO_DATA;
            } else if (file.bands[atlaas::VARIANCE][id] > vt) {
                file.bands[atlaas::Z_MEAN][id] = file.bands[atlaas::Z_MAX][id];
            }
        }
        file.bands = {file.bands[atlaas::Z_MEAN]};
        file.names = {"DTM"};
    }
}

inline void decimate(gdalwrap::gdal& region, size_t scale = 5) {
    const size_t sx = region.get_width(), sy = region.get_height();
    auto it = region.bands[0].begin(), begin = it;
    for (size_t i = 0; i < sy; i+=scale) {
        for (size_t j = 0; j < sx; j+=scale, it++) {
            *it = *(begin + j + i * sx);
            for (size_t u = 0; u < scale; u++) {
                for (size_t v = 0; v < scale; v++) {
                    *it = std::max(*it, *(begin + j + v + (i + u) * sx));
                }
            }
        }
    }
    region.set_transform(region.get_utm_pose_x(), region.get_utm_pose_y(),
        region.get_scale_x() * scale, region.get_scale_y() * scale);
    region.set_size(1, sx / scale, sy / scale);
}

inline void edge(gdalwrap::gdal& region, float factor = EDGE_FACTOR) {
    for (auto it = region.bands[0].begin(), it1 = it + 1,
            it2 = it + region.get_width(), it3 = it2 + 1,
            end = region.bands[0].end(); it3 < end; it++,
            it1++, it2++, it3++) {
        *it = (*it > NO_DATA) ? std::min(255.0f, 1 + factor *
            (std::abs(*it - *it1) +
             std::abs(*it - *it2) +
             std::abs(*it - *it3) )) : 0;
    }
}

inline void region(std::vector<gdalwrap::gdal>& tiles, const std::string& filepath) {
    // select only dtm band
    select(tiles);
    // merge all tiles
    gdalwrap::gdal result = gdalwrap::merge(tiles, NO_DATA);
    // edge detect
    edge(result);
    // scale down
    decimate(result);
    // convert to grayscale PNG
    std::vector<uint8_t> gray(result.bands[0].begin(), result.bands[0].end());
    result.export8u(filepath, gray, "PNG");
}

inline void glob_region(const std::string& pattern_in,  const std::string& file_out) {
    std::vector<std::string> files = glob(pattern_in);
    std::vector<gdalwrap::gdal> tiles(files.size());
    auto it = tiles.begin();
    for (const std::string& file : files) {
        (*it++).load(file);
    }
    region(tiles, file_out);
}

} // namespace atlaas

#endif // ATLAAS_REGION_HPP
