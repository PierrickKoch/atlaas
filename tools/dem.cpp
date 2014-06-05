#include <iostream>
#include <atlaas/atlaas.hpp>
#include <gdalwrap/gdal.hpp>

int main(int argc, char * argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " atlaas.tif dem.tif" << std::endl;
        return 1;
    }
    gdalwrap::gdal geotiff( argv[1] );
    auto band = geotiff.bands[ atlaas::Z_MEAN ];
    for (size_t idx = 0; idx < band.size(); idx++)
        if ( geotiff.bands[ atlaas::N_POINTS ][idx] < 1 )
            band[idx] = -10000;
    geotiff.bands = { band };
    geotiff.names = {"ATLAAS_DEM"};
    geotiff.save( argv[2] );
    return 0;
}
