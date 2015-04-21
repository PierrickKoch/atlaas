#include <vector>
#include <iostream>
#include <atlaas/region.hpp>
#include <gdalwrap/gdal.hpp>

int main(int argc, char * argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " file1.tif ... out.png"
                  << std::endl;
        return 1;
    }

    std::vector<gdalwrap::gdal> files(argc - 2);
    // open files
    for (int filen = 1; filen < argc - 1; filen++) {
        files[filen - 1].load( argv[filen] );
    }
    atlaas::region(files, argv[argc - 1]);
    return 0;
}
