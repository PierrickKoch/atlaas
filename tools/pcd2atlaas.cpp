#include <cstdlib>
#include <iostream>
#include <atlaas/io.hpp>
#include <atlaas/pcd.hpp>

int main(int argc, char * argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " in.pcd out.apc" << std::endl;
        return EXIT_FAILURE;
    }
    atlaas::points cloud;
    atlaas::matrix transformation;
    try {
        atlaas::read_pcd(argv[1], cloud, transformation);
        atlaas::write_raw(argv[2], cloud, transformation);
    } catch (std::exception& e) {
        std::cerr << "[error] " << e.what();
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
