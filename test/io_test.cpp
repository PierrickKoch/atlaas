#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/io.hpp>
#include <tests.hpp>

int main(int argc, char * argv[]) {
    std::cout << "atlaas io test..." << std::endl;

    atlaas::points cloud = atlaas::random_cloud();
    atlaas::matrix transformation = atlaas::identity;

    atlaas::write_raw("test.scan", cloud, transformation);

    atlaas::points cloud2;
    atlaas::matrix transformation2;

    atlaas::read_raw("test.scan", cloud2, transformation2);

    assert( atlaas::allclose( cloud, cloud2 ) );
    assert( atlaas::allclose( transformation, transformation2 ) );

    std::cout << "done." << std::endl;
    return 0;
}
