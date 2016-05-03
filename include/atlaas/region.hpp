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
#include <algorithm> // std::count_if
#include <atlaas/common.hpp>
#include <gdalwrap/gdal.hpp>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_multifit.h>

namespace atlaas {

static const float VARIANCE_THRESHOLD = 0.01;
static const float EDGE_FACTOR = 100;
static const float NO_DATA = -10000;
static const size_t DEFAULT_SCALE = 10;

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
            file.bands[atlaas::N_POINTS],// XXX
            file.bands[atlaas::Z_MIN],
            file.bands[atlaas::Z_MAX],
            file.bands[atlaas::VARIANCE],
        };
        file.names = {"DTM", "DIST_SQ", "TIME"};
    }
}

/**
 * a,b,c define plan f(x,y,z) = a*x + b*y + c
 * chisq: sum squared diff between points and plan(a,b,c)
 */
inline float class_cell(double a, double b, double c, double chisq) {
    // Nz = 1 / np.sqrt(a**2 + b**2 + 1) # normal in Z
    // T = np.arccos(Nz) # normal angle
    return a*a + b*b + chisq; // TODO
}

class lstsq {
    gsl_matrix *_X;
    gsl_vector *_y;
    gsl_vector *_beta;
    gsl_matrix *_cov;
    gsl_multifit_linear_workspace *_work;
public:
    lstsq(size_t sz_rt) {
        size_t sz = sz_rt * sz_rt;
        this->_X = gsl_matrix_calloc(sz, 3);
        this->_y = gsl_vector_alloc(sz);
        this->_beta = gsl_vector_alloc(3);
        this->_cov = gsl_matrix_alloc(3, 3);
        this->_work = gsl_multifit_linear_alloc(sz, 3);
        for (size_t i = 0, id = 0; i < sz_rt; i++) {
            for (size_t j = 0; j < sz_rt; j++, id++) {
                gsl_matrix_set(this->_X, id, 0, j);
                gsl_matrix_set(this->_X, id, 1, i);
                gsl_matrix_set(this->_X, id, 2, 1);
            }
        }
    }
    ~lstsq() {
        gsl_matrix_free(this->_X);
        gsl_vector_free(this->_y);
        gsl_matrix_free(this->_cov);
        gsl_vector_free(this->_beta);
        gsl_multifit_linear_free(this->_work);
    }
    /**
     * input: list of z values
     * a,b,c: output normal vector
     * chisq: sum of square diff, residual
     */
    void multifit_linear(const std::vector<float>& input,
            double* a, double* b, double* c, double* chisq) {
        for (size_t id = 0; id < input.size(); id++)
            gsl_vector_set(this->_y, id, input[id]);
        gsl_multifit_linear(this->_X, this->_y, this->_beta, this->_cov,
            chisq, this->_work);
        *a = gsl_vector_get(this->_beta, 0);
        *b = gsl_vector_get(this->_beta, 1);
        *c = gsl_vector_get(this->_beta, 2);
    }
};

inline void multifit_linear_no_data(size_t sz_rt, size_t no_data,
const std::vector<float>& input, double* a, double* b, double* c,
double* chisq) {
    gsl_matrix *_X;
    gsl_vector *_y;
    gsl_vector *_beta;
    gsl_matrix *_cov;
    gsl_multifit_linear_workspace *_work;
    // init
    size_t sz = (sz_rt * sz_rt) - no_data;
    _X = gsl_matrix_calloc(sz, 3);
    _y = gsl_vector_alloc(sz);
    _beta = gsl_vector_alloc(3);
    _cov = gsl_matrix_alloc(3, 3);
    _work = gsl_multifit_linear_alloc(sz, 3);
    for (size_t i = 0, od = 0, id = 0; i < sz_rt; i++) {
        for (size_t j = 0; j < sz_rt; j++, id++) {
            if (input[id] > NO_DATA) {
                gsl_matrix_set(_X, od, 0, j);
                gsl_matrix_set(_X, od, 1, i);
                gsl_matrix_set(_X, od, 2, 1);
                gsl_vector_set(_y, od, input[id]);
                od++;
            }
        }
    }
    // fit
    gsl_multifit_linear(_X, _y, _beta, _cov, chisq, _work);
    *a = gsl_vector_get(_beta, 0);
    *b = gsl_vector_get(_beta, 1);
    *c = gsl_vector_get(_beta, 2);
    // free
    gsl_matrix_free(_X);
    gsl_vector_free(_y);
    gsl_matrix_free(_cov);
    gsl_vector_free(_beta);
    gsl_multifit_linear_free(_work);
}

/**
 * Decimate least-sqaure fitting on scale*scale patch of z-mean values
 */
inline void decimate(gdalwrap::gdal& region, size_t scale = DEFAULT_SCALE) {
    const size_t sx = region.get_width(), sy = region.get_height();
    auto it_gray = region.bands[0].begin(), begin_gray = it_gray,
         it_alph = region.bands[1].begin(), begin_alph = it_alph,
         it_time = region.bands[2].begin(), begin_time = it_time,
         it_a = region.bands[3].begin(),
         it_b = region.bands[4].begin(),
         it_c = region.bands[5].begin(),
         it_d = region.bands[6].begin();
    float dist, min_dist;
    lstsq planar_fit(scale);
    size_t lstsqid;
    double a,b,c,chisq;
    std::vector<float> lstsqin(scale*scale);
    for (size_t i = 0; i < sy; i+=scale) {
        for (size_t j = 0; j < sx; j+=scale, it_gray++, it_alph++, it_time++,
                it_a++, it_b++, it_c++, it_d++) {
            *it_alph = 0;
            lstsqid = 0;
            min_dist = std::numeric_limits<float>::max();
            for (size_t u = 0; u < scale; u++) {
                for (size_t v = 0; v < scale; v++) {
                    size_t p = j + v + (i + u) * sx;
                    dist = *(begin_alph + p);
                    if (dist > NO_DATA)
                        *it_alph += dist;
                    lstsqin[lstsqid++] = *(begin_gray + p);
                    if (dist < min_dist) {
                        min_dist = dist;
                        *it_time = *(begin_time + p);
                    }
                }
            }
            size_t no_data = 0;
            // count no_data
            for (float v : lstsqin)
                if (v <= NO_DATA)
                    no_data++;
            if (no_data) {
                if (no_data > (scale*scale/2)) {
                    *it_gray = 255;
                    *it_alph = 255;
                    *it_a = 0;
                    *it_b = 0;
                    *it_c = 0;
                    *it_d = 0;
                    continue;
                } else {
                    multifit_linear_no_data(scale, no_data, lstsqin, &a,&b,&c,
                        &chisq);
                }
            } else {
                planar_fit.multifit_linear(lstsqin, &a,&b,&c,&chisq);
            }
            *it_a = a;
            *it_b = b;
            *it_c = c;
            *it_d = chisq;
            *it_gray = class_cell(a,b,c,chisq);
            *it_alph /= scale*scale;
        }
    }
    region.set_transform(region.get_utm_pose_x(), region.get_utm_pose_y(),
        region.get_scale_x() * scale, region.get_scale_y() * scale);
    region.set_size(region.bands.size(), sx / scale, sy / scale);
}

inline void tile_to_region(gdalwrap::gdal& tile, const std::string& filepath,
        float vt = VARIANCE_THRESHOLD, size_t scale = DEFAULT_SCALE) {
    // prepare data
    for (size_t id = 0; id < tile.bands[0].size(); id++) {
        if (tile.bands[atlaas::N_POINTS][id] < 1) {
            tile.bands[atlaas::Z_MEAN][id] = NO_DATA;
            tile.bands[atlaas::DIST_SQ][id] = NO_DATA;
            tile.bands[atlaas::TIME][id] = NO_DATA;
        } else {
            tile.bands[atlaas::DIST_SQ][id] /= 20; // f(x) = x^2 / 20
            if (tile.bands[atlaas::VARIANCE][id] > vt) {
                tile.bands[atlaas::Z_MEAN][id] =
                    tile.bands[atlaas::Z_MAX][id];
            }
        }
    }
    tile.bands = {
        tile.bands[atlaas::Z_MEAN],
        tile.bands[atlaas::DIST_SQ],
        tile.bands[atlaas::TIME],
        tile.bands[atlaas::N_POINTS],// XXX
        tile.bands[atlaas::Z_MIN],
        tile.bands[atlaas::Z_MAX],
        tile.bands[atlaas::VARIANCE],
    };
    tile.names = {"DTM", "DIST_SQ", "TIME"};
    decimate(tile, scale);
    // convert to grayscale PNG
    std::vector<uint8_t> gray(tile.bands[0].begin(), tile.bands[0].end());
    std::vector<uint8_t> alph(tile.bands[1].size());
    std::transform(tile.bands[1].begin(), tile.bands[1].end(), alph.begin(),
        [](float v) -> uint8_t { return v > 255 ? 0 : v < 0 ? 0 : 255 - v; });
    // tile set meta COVERAGE = band(alpha=precision=id2 where > 1) / size
    float coverage = std::count_if(alph.begin(), alph.end(),
        [](float f) { return f > 0; });
    coverage /= (float) tile.bands[1].size();
    tile.metadata["COVERAGE"] = std::to_string(coverage);
    tile.metadata["AVGALPHA"] = std::to_string(average(tile.bands[1]));
    tile.export8u(filepath, {gray, alph}, "PNG");
}
inline void tile_to_region_io(const std::string& in, const std::string& out,
        float vt = VARIANCE_THRESHOLD, size_t scale = DEFAULT_SCALE) {
    auto tile = gdalwrap::gdal(in);
    tile_to_region(tile, out, vt, scale);
}

/**
 * Region converts a list of tiles into a 2 layers PNG file,
 * 1st is 8 bit grayscale representation of traversability:
 *  0 = unknown, [1..255] roughness, 1 = flat, 255 = obstacle.
 * 2nd is 8 bit alpha/transparency representation of uncertainty:
 *  [0..255] precision, 0 = high uncertainty, 255 = low uncertainty.
 * PNG comes with .aux.xml file containing georeferenced metadata (GDAL).
 */
inline void region(std::vector<gdalwrap::gdal>& tiles,
                   const std::string& filepath) {
    if (tiles.size() < 1) return;
    // select only dtm band
    select(tiles);
    // merge all tiles
    gdalwrap::gdal result = gdalwrap::merge(tiles, NO_DATA);
    // scale down
    decimate(result);
    // convert to grayscale PNG
    std::vector<uint8_t> gray(result.bands[0].begin(), result.bands[0].end());
    std::vector<uint8_t> alph(result.bands[1].size());
    std::transform(result.bands[1].begin(), result.bands[1].end(), alph.begin(),
        [](float v) -> uint8_t { return v > 255 ? 0 : v < 0 ? 0 : 255 - v; });
    result.export8u(filepath, {gray, alph}, "PNG");
    gdalwrap::gdal copy(result);
    copy.bands = {copy.bands[3], copy.bands[4], copy.bands[5], copy.bands[6]};
    copy.names = {"a", "b", "c", "chisq"};
    copy.save(filepath+".abc_chisq.tif");
    result.bands = {result.bands[2]};
    result.names = {"TIME"};
    result.save(filepath+".time.tif");
}

/**
 * Same as region, to which we pass a patern to get the list of tiles.
 * See: region() and glob()
 */
inline void glob_region(const std::string& pattern_in,
                        const std::string& file_out) {
    std::vector<std::string> files = glob(pattern_in);
    if (files.size() < 1) return;
    std::vector<gdalwrap::gdal> tiles(files.size());
    auto it = tiles.begin();
    for (const std::string& file : files) {
        (*it++).load(file);
    }
    region(tiles, file_out);
}

inline void merge_io(const std::string& pattern_in,
                     const std::string& file_out, bool compress = false) {
    std::vector<std::string> files = glob(pattern_in);
    if (files.size() < 1) return;
    std::vector<gdalwrap::gdal> tiles(files.size());
    auto it = tiles.begin();
    for (const std::string& file : files) {
        (*it++).load(file);
    }
    gdalwrap::gdal result = gdalwrap::merge(tiles, 0);
    // result set meta COVERAGE = band(alpha=precision=id2 where > 1) / size
    float coverage = std::count_if(result.bands[1].begin(), result.bands[1].end(),
        [](float f) { return f > 0; }) / (float) result.bands[1].size();
    result.metadata["COVERAGE"] = std::to_string(coverage);
    result.metadata["AVGALPHA"] = std::to_string(average(result.bands[1]));
    std::string ext = gdalwrap::toupper( file_out.substr( file_out.rfind(".") + 1 ) );
    if (!ext.compare("PNG")) {
        result.export8u(file_out, { gdalwrap::raster2bytes(result.bands[0]),
                gdalwrap::raster2bytes(result.bands[1]) }, "PNG");
    } else {
        result.save(file_out, compress);
    }
}

} // namespace atlaas

#endif // ATLAAS_REGION_HPP
