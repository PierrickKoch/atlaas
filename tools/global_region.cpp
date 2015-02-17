#include <iostream>
#include <atlaas/atlaas.hpp>
#include <gdalwrap/gdal.hpp>

static const float THRES_VARIANCE   = 0.5;
static const float THRES_ROUGH      = 0.1; // 3 * 0.1m
static const float THRES_OBSTACLE   = 0.9; // 3 * 0.2m
static const float VAL_OBSTACLE     = 255;
static const float VAL_ROUGH        = 100;
static const float VAL_FLAT         = 0;

void select(std::vector<gdalwrap::gdal>& files) {
    for (gdalwrap::gdal& file : files) {
        for (size_t id = 0; id < file.bands[0].size(); id++) {
            if (file.bands[atlaas::N_POINTS][id] < 1) {
                file.bands[atlaas::Z_MEAN][id] = -10000;
            } else if (file.bands[atlaas::VARIANCE][id] > THRES_VARIANCE) {
                file.bands[atlaas::Z_MEAN][id] = file.bands[atlaas::Z_MAX][id];
            }
        }
        file.bands = {file.bands[atlaas::Z_MEAN]};
        file.names = {"DTM"};
    }
}

void scale(std::vector<gdalwrap::gdal>& files) {
    // TODO
}

std::vector<uint8_t> edge(const gdalwrap::raster& r, size_t sx, size_t sy) {
    std::vector<uint8_t> bytes(r.size());
    for (size_t ix = 0; ix < sx - 1; ix++)
    for (size_t iy = 0; iy < sy - 1; iy++) {
        float di = std::abs(r[ix + iy*sx] - r[ix + iy * sx + 1])
                 + std::abs(r[ix + iy*sx] - r[ix + (iy + 1) * sx])
                 + std::abs(r[ix + iy*sx] - r[ix + (iy + 1) * sx + 1]);
        bytes[ix + iy*sx] = (di > THRES_ROUGH) ? (
            (di > THRES_OBSTACLE) ? VAL_OBSTACLE : VAL_ROUGH ) : VAL_FLAT;
    }
    return bytes;
}

int main(int argc, char * argv[]) {
    if (argc < 4) {
        std::cerr << "usage: " << argv[0] << " file1.tif file2.tif ... out.tif"
                  << std::endl;
        return 1;
    }

    std::vector<gdalwrap::gdal> files(argc - 2);
    // open files
    for (int filen = 1; filen < argc - 1; filen++) {
        files[filen - 1].load( argv[filen] );
    }
    // select only dtm band
    select(files);
    // scale down 10% (to a metric scale)
    scale(files);
    // merge all files
    gdalwrap::gdal result = merge(files);
    // edge detect / sobel like
    result.export8u(argv[argc - 1],
        edge(result.bands[0], result.get_width(), result.get_height()), "PNG");
    return 0;
}
