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

#ifdef NDEBUG
#define LOG(x)
#else
#define LOG(x) std::cout << (x) ;
#endif

namespace atlaas {

static const float VARIANCE_THRESHOLD = 0.01;
static const float EDGE_FACTOR = 100;
static const float NO_DATA = -10000;

/**
 * C++ version of POSIX glob() function, see `man 3 glob`
 */
inline std::vector<std::string> glob(const std::string& pattern) {
    glob_t result;
    std::vector<std::string> match;
    // might add |GLOB_BRACE for "{0-9}*x{0-9}*"
    // and |GLOB_TILDE for ~/path/from/home
    if (!glob(pattern.c_str(), GLOB_NOSORT, NULL, &result)) {
        match.assign(result.gl_pathv, result.gl_pathv + result.gl_pathc);
    }
    globfree(&result);
    return match;
}

/**
 * Keep only band of interest in each tiles,
 * before to merge them, save memory and cpu.
 */
inline void select(std::vector<gdalwrap::gdal>& files,
                   float vt = VARIANCE_THRESHOLD) {
    // time delta
    long dt, t_max = 0;
    for (gdalwrap::gdal& file : files) {
        t_max = std::max(t_max, std::stol(file.get_meta("TIME", "0")));
    }
    for (gdalwrap::gdal& file : files) {
        dt = t_max - std::stol(file.get_meta("TIME", "0"));
        file.metadata["TIME"] = std::to_string(t_max);
        for (size_t id = 0; id < file.bands[0].size(); id++) {
            if (file.bands[atlaas::N_POINTS][id] < 1) {
                file.bands[atlaas::Z_MEAN][id] = NO_DATA;
                file.bands[atlaas::DIST_SQ][id] = NO_DATA;
                file.bands[atlaas::TIME][id] = NO_DATA;
            } else {
                file.bands[atlaas::DIST_SQ][id] /= 20; // f(x) = x^2 / 20
                if (dt) {
                    file.bands[atlaas::TIME][id] -= dt;
                }
                if (file.bands[atlaas::VARIANCE][id] > vt) {
                    file.bands[atlaas::Z_MEAN][id] =
                        file.bands[atlaas::Z_MAX][id];
                }
            }
        }
        file.bands = {
            file.bands[atlaas::Z_MEAN],
            file.bands[atlaas::DIST_SQ],
            file.bands[atlaas::TIME],
        };
        file.names = {"DTM", "DIST_SQ", "TIME"};
    }
}

/**
 * Conservative decimate keeping max "edge" value
 */
inline void decimate(gdalwrap::gdal& region, size_t scale = 5) {
    const size_t sx = region.get_width(), sy = region.get_height();
    auto it_gray = region.bands[0].begin(), begin_gray = it_gray,
         it_alph = region.bands[1].begin(), begin_alph = it_alph,
         it_time = region.bands[2].begin(), begin_time = it_time;
    for (size_t i = 0; i < sy; i+=scale) {
        for (size_t j = 0; j < sx; j+=scale, it_gray++, it_alph++, it_time++) {
            *it_gray = *(begin_gray + j + i * sx);
            *it_alph = *(begin_alph + j + i * sx);
            *it_time = *(begin_time + j + i * sx);
            for (size_t u = 0; u < scale; u++) {
                for (size_t v = 0; v < scale; v++) {
                    size_t p = j + v + (i + u) * sx;
                    if (*(begin_gray + p) > *it_gray) {
                        *it_gray = *(begin_gray + p);
                        *it_alph = *(begin_alph + p);
                        *it_time = *(begin_time + p);
                    }
                }
            }
        }
    }
    region.set_transform(region.get_utm_pose_x(), region.get_utm_pose_y(),
        region.get_scale_x() * scale, region.get_scale_y() * scale);
    region.set_size(region.bands.size(), sx / scale, sy / scale);
}

/**
 * Gradient will sum absolute differences of next neighbours, in place.
 */
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

inline std::vector<std::vector<uint8_t>>
vf32_4vu8(gdalwrap::gdal& region, size_t band) {
    const size_t sx = region.get_width(), sy = region.get_height();
    long t_ref = std::stol(region.get_meta("TIME", "0"));
    std::vector<std::vector<uint8_t>> argb(4);
    for (auto& layer : argb) {
        layer.resize(sx * sy);
    }
    // for ... a = (char) vf32 & 0xff, b = (char) vf32 & 0xff00 << 2, ...;
    return argb;
}

/**
 * Region converts a list of tiles into a 2 layers PNG file,
 * 1st is 8 bit grayscale representation of traversability:
 *  0 = unknown, [1..255] roughness, 1 = flat, 255 = obstacle.
 * 2nd is 8 bit alpha/transparency representation of uncertainty:
 *  [0..255] confidence, 0 = high uncertainty, 255 = low uncertainty.
 * PNG comes with .aux.xml file containing georeferenced metadata (GDAL).
 */
inline void region(std::vector<gdalwrap::gdal>& tiles,
                   const std::string& filepath) {
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
    std::vector<uint8_t> alph(result.bands[1].size());
    std::transform(result.bands[1].begin(), result.bands[1].end(), alph.begin(),
        [](float v) -> uint8_t { return v > 255 ? 0 : v < 0 ? 0 : 255 - v; });
    result.export8u(filepath, {gray, alph}, "PNG");
    gdalwrap::gdal r2;
    r2.copy_meta(result);
    r2.bands = {result.bands[2]};
    r2.names = {"TIME"};
    r2.save(filepath+".time.tif");
    //result.export8u(filepath+".time.png", vf32_4vu8(result, 2), "PNG");
}

/**
 * Same as region, to which we pass a patern to get the list of tiles.
 * See: region() and glob()
 */
inline void glob_region(const std::string& pattern_in,
                        const std::string& file_out) {
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
