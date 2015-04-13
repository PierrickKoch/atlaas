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
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_multifit.h>

namespace atlaas {

static const float VARIANCE_THRESHOLD = 0.01;
static const float EDGE_FACTOR = 100;
static const float NO_DATA = -10000;
static const size_t DEFAULT_SCALE = 5;

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
 * a,b,c define plan f(x,y,z) = a*x + b*y + c
 * chisq: sum squared diff between points and plan(a,b,c)
 */
inline float class_cell(double a, double b, double c, double chisq) {
    return a * b + chisq; // TODO
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
                gsl_matrix_set(this->_X, id, 0, i);
                gsl_matrix_set(this->_X, id, 1, j);
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

/**
 * Decimate least-sqaure fitting on scale*scale patch of z-mean values
 */
inline void decimate(gdalwrap::gdal& region, size_t scale = DEFAULT_SCALE) {
    const size_t sx = region.get_width(), sy = region.get_height();
    auto it_gray = region.bands[0].begin(), begin_gray = it_gray,
         it_alph = region.bands[1].begin(), begin_alph = it_alph,
         it_time = region.bands[2].begin(), begin_time = it_time;
    float dist, min_dist;
    lstsq planar_fit(scale);
    size_t lstsqid;
    double a,b,c,chisq;
    std::vector<float> lstsqin(scale*scale);
    for (size_t i = 0; i < sy; i+=scale) {
        for (size_t j = 0; j < sx; j+=scale, it_gray++, it_alph++, it_time++) {
            *it_alph = 0;
            lstsqid = 0;
            min_dist = std::numeric_limits<float>::max();
            for (size_t u = 0; u < scale; u++) {
                for (size_t v = 0; v < scale; v++) {
                    size_t p = j + v + (i + u) * sx;
                    dist = *(begin_alph + p);
                    *it_alph += dist;
                    lstsqin[lstsqid++] = *(begin_gray + p);
                    if (dist < min_dist) {
                        min_dist = dist;
                        *it_time = *(begin_time + p);
                    }
                }
            }
            planar_fit.multifit_linear(lstsqin, &a,&b,&c,&chisq);
            *it_gray = class_cell(a,b,c,chisq);
            *it_alph /= scale*scale;
        }
    }
    region.set_transform(region.get_utm_pose_x(), region.get_utm_pose_y(),
        region.get_scale_x() * scale, region.get_scale_y() * scale);
    region.set_size(region.bands.size(), sx / scale, sy / scale);
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
    // scale down
    decimate(result);
    // convert to grayscale PNG
    std::vector<uint8_t> gray(result.bands[0].begin(), result.bands[0].end());
    std::vector<uint8_t> alph(result.bands[1].size());
    std::transform(result.bands[1].begin(), result.bands[1].end(), alph.begin(),
        [](float v) -> uint8_t { return v > 255 ? 0 : v < 0 ? 0 : 255 - v; });
    result.export8u(filepath, {gray, alph}, "PNG");
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
    std::vector<gdalwrap::gdal> tiles(files.size());
    auto it = tiles.begin();
    for (const std::string& file : files) {
        (*it++).load(file);
    }
    region(tiles, file_out);
}

} // namespace atlaas

#endif // ATLAAS_REGION_HPP
