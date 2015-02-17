#include <iostream>
#include <atlaas/atlaas.hpp>
#include <gdalwrap/gdal.hpp>

static const float VARIANCE_THRESHOLD = 0.01;
static const float EDGE_FACTOR = 1337;
static const float NO_DATA = -10000;

void select(std::vector<gdalwrap::gdal>& files, float vt = VARIANCE_THRESHOLD) {
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

void scale(std::vector<gdalwrap::gdal>& files) {
    // TODO scale down 10% (to a metric scale)q
}

std::vector<uint8_t> edge(const gdalwrap::raster& r, size_t sx, size_t sy,
        float factor = EDGE_FACTOR) {
    size_t cx = 0;
    float r1, r2, r3;
    std::vector<uint8_t> bytes(r.size());
    auto bit = bytes.begin();
    for (auto rit = r.begin(), end = r.end() - sx - 1; rit < end; rit++, bit++) {
        if (cx == sx - 1) {
            cx = 0;
        } else if (*rit > NO_DATA) {
            cx++;
            r1 = *rit - *(rit + 1);
            r2 = *rit - *(rit + sx);
            r2 = *rit - *(rit + sx + 1);
            *bit = std::min(255.0f, 1 + (r1*r1 + r2*r2 + r3*r3) * factor);
        }
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
    // TODDO scale(files);
    // merge all files
    gdalwrap::gdal result = merge(files);
    // edge detect / sobel like
    result.export8u(argv[argc - 1],
        edge(result.bands[0], result.get_width(), result.get_height()), "PNG");
    return 0;
}
