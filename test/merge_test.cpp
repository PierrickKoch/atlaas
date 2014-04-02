#include <cassert>
#include <iostream>
#include <atlaas/atlaas.hpp>
#include <gdalwrap/gdal.hpp>

void assert_meta_xy(const gdalwrap::gdal& meta, double x, double y) {
    assert( meta.get_utm_pose_x() == x );
    assert( meta.get_utm_pose_y() == y );
}

int main(int argc, char * argv[]) {
    std::cout << "atlaas merge/slide test..." << std::endl;

    atlaas::atlaas test;
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, true);
    // identity matrix
    atlaas::matrix transformation = {{
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1,
    }};
    atlaas::points cloud = {{
        {{ 1, 2, 3}},
        {{-1,-2,-3}},
    }};

    assert_meta_xy(test.get_meta(), 0, 0);

    test.merge(cloud, transformation);

    assert_meta_xy(test.get_meta(), -40, 40);

    std::cout << "done." << std::endl;
    return 0;
}
