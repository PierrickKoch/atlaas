#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/atlaas.hpp>
#include <atlaas/region.hpp>
#include <gdalwrap/gdal.hpp>
#include <tests.hpp>
#include <chrono>
#include <thread>

static const size_t width  = 100;
static const size_t height = 100;

int main(int argc, char * argv[]) {
    std::cout << "atlaas region test..." << std::endl;

    gdalwrap::gdal result = gdalwrap::gdal();
    std::vector<uint64_t> pcd_ts(width * height);
    for (size_t i = 0; i < pcd_ts.size(); i++)
        pcd_ts[i] = i;
    result.set_size(1, width, height);
    for (size_t i = 0; i < result.get_width(); i++)
    for (size_t j = 0; j < result.get_height(); j++)
        result.bands[0][i*width + j] = i*width + j;
    result.export8u("region_time.png",
        atlaas::encode_id_to_rgba(result, 0, pcd_ts), "PNG");

    // TODO
    // open png
    // assert png[i*width + j] == i*width + j


    atlaas::atlaas test;
    atlaas::points cloud = atlaas::random_cloud();
    atlaas::matrix transformation = atlaas::identity;
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, true);
    for (size_t i = 0; i < 100; i++) {
        transformation[3] = i;
        transformation[7] = i;
        test.merge(cloud, transformation);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    test.region("region_time2.png");

    std::cout << "done." << std::endl;
    return 0;
}
