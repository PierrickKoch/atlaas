#include <cmath>
#include <limits>
#include <iostream>
#include <atlaas/atlaas.hpp>
#include <gdalwrap/gdal.hpp>

bool same(double a, double b) {
    return std::abs(a-b) < std::numeric_limits<double>::epsilon();
}

int main(int argc, char * argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " file1.tif file2.tif ..." << std::endl;
        return 1;
    }
    double scale_x, scale_y, utm_x, utm_y,
           min_utm_x, max_utm_x,
           min_utm_y, max_utm_y;
    size_t width, height, bsize;
    std::vector<gdalwrap::gdal> files(argc - 1);
    {
        std::string filepath( argv[1] );
        files[0].load(filepath);
        scale_x = files[0].get_scale_x();
        scale_y = files[0].get_scale_y();
        utm_x = files[0].get_utm_pose_x();
        utm_y = files[0].get_utm_pose_y();
        width = files[0].get_width();
        height = files[0].get_height();
        bsize = files[0].bands.size();
        min_utm_x = max_utm_x = utm_x;
        min_utm_y = max_utm_y = utm_y;
    }
    for (int filen = 2; filen < argc; filen++) {
        std::string filepath( argv[filen] );
        files[filen-1].load(filepath);
        if (same(scale_x, files[filen-1].get_scale_x()) and
            same(scale_y, files[filen-1].get_scale_y()) and
            same(width, files[filen-1].get_width()) and
            same(height, files[filen-1].get_height()) and
            same(bsize, files[filen-1].bands.size()) ) {
            // get min/max
            utm_x = files[filen-1].get_utm_pose_x();
            utm_y = files[filen-1].get_utm_pose_y();
            if (utm_x < min_utm_x) min_utm_x = utm_x;
            if (utm_y < min_utm_y) min_utm_y = utm_y;
            if (utm_x > max_utm_x) max_utm_x = utm_x;
            if (utm_y > max_utm_y) max_utm_y = utm_y;
        } else {
            std::cerr << "[error] merge only same sized tiles: " << filepath << std::endl;
        }
    }
    double ulx = min_utm_x,
           uly = max_utm_y,
           lrx = max_utm_x + scale_x * width,
           lry = min_utm_y + scale_y * height;
    size_t sx = std::floor((lrx - ulx) / scale_x + 0.5),
           sy = std::floor((lry - uly) / scale_y + 0.5);
    gdalwrap::gdal result;
    result.set_transform(ulx, uly, scale_x, scale_y);
    result.set_size(bsize, sx, sy);

    for (const gdalwrap::gdal& file : files) {
        utm_x = file.get_utm_pose_x();
        utm_y = file.get_utm_pose_y();
        int xoff = std::floor( (utm_x - ulx) / scale_x + 0.1 );
        int yoff = std::floor( (utm_y - uly) / scale_y + 0.1 );
        size_t start = xoff + yoff * sx;
        for (int band = 0; band < bsize; band ++) {
            // copy file.bands[band] into result.bands[band]
            auto it2 = result.bands[band].begin() + start;
            for (auto it1 = file.bands[band].begin();
                it1 < file.bands[band].end();
                it1 += width, it2 += sx) {
                std::copy(it1, it1+width, it2);
            }
        }
    }

    result.save("out.tif");
    return 0;
}
