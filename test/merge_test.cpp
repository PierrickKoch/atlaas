#include <iostream>
#include <atlaas/atlaas.hpp>
#include <gdalwrap/gdal.hpp>

int main(int argc, char * argv[]) {
    std::cout << "atlaas test..." << std::endl;

    atlaas::atlaas test;
    test.init(120.0, 120.0, 0.1, 1.0, 1.0, 1.0, 31, true);
    atlaas::matrix transformation = {{
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1 }};
    atlaas::points cloud(2);
    cloud[0] = {{ 1, 2, 3}};
    cloud[1] = {{-1,-2,-3}};
    test.merge(cloud, transformation);

    std::cout << "done." << std::endl;
    return 0;
}
