#include <iostream>
#include <atlaas/common.hpp>
#include <gdalwrap/gdal.hpp>

static const float VARIANCE_THRESHOLD = 0.01;
static const float EDGE_FACTOR = 100;
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

void decimate(gdalwrap::gdal& region, float scale = 5) {
    size_t sx = region.get_width(), sy = region.get_height();
    auto it = region.bands[0].begin(), new_it = it;
    for (size_t i = 0; i < sy; i+=scale) {
        for (size_t j = 0; j < sx; j+=scale, it++) {
            for (int u = 0; u < scale; u++) {
                for (int v = 0; v < scale; v++) {
                    *it = std::max(*it, *(new_it + j + v + (i + u) * sy));
                }
            }
        }
    }
    region.set_transform(region.get_utm_pose_x(), region.get_utm_pose_y(),
        region.get_scale_x() * scale, region.get_scale_y() * scale);
    region.set_size(1, sx / scale, sy / scale);
}

void edge(gdalwrap::gdal& region, float factor = EDGE_FACTOR) {
    size_t sx = region.get_width();
    for (auto rit = region.bands[0].begin(), rit1 = rit + 1, rit2 = rit + sx,
            rit3 = rit2 + 1, end = region.bands[0].end(); rit3 < end; rit++,
            rit1++, rit2++, rit3++) {
        if (*rit > NO_DATA) {
            *rit = std::min(255.0f, 1 + factor *
                (std::abs(*rit - *rit1) +
                 std::abs(*rit - *rit2) +
                 std::abs(*rit - *rit3) ));
        } else {
            *rit = 0;
        }
    }
}

void region(std::vector<gdalwrap::gdal>& files, const std::string filepath) {
    // select only dtm band
    select(files);
    // merge all files
    gdalwrap::gdal result = gdalwrap::merge(files, NO_DATA);
    // edge detect
    edge(result);
    // scale down
    decimate(result);
    // convert to grayscale PNG
    std::vector<uint8_t> gray(result.bands[0].begin(), result.bands[0].end());
    result.export8u(filepath, gray, "PNG");
}

int main(int argc, char * argv[]) {
    if (argc < 4) {
        std::cerr << "usage: " << argv[0] << " file1.tif file2.tif ... out.png"
                  << std::endl;
        return 1;
    }

    std::vector<gdalwrap::gdal> files(argc - 2);
    // open files
    for (int filen = 1; filen < argc - 1; filen++) {
        files[filen - 1].load( argv[filen] );
    }
    region(files, argv[argc - 1]);
    return 0;
}
