#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/io.hpp>
#include <tests.hpp>

int main(int argc, char * argv[]) {
    std::cout << "atlaas io test..." << std::endl;

    atlaas::points cloud = atlaas::random_cloud();
    atlaas::matrix transformation = atlaas::identity;

    size_t length = cloud.size();
    size_t length2 = cloud[0].size();
    atlaas::c_save("test.scan", (const float*)cloud.data(), length,
        length2, transformation.data());

    double transformation2[16];
    float* cloud2 = atlaas::c_load("test.scan", length, transformation2);
    assert(length == cloud.size());
    for (size_t i = 0; i < length; i++) {
        for (size_t j = 0; j < length2; j++) {
            assert( atlaas::allclose( cloud2[i*length2+j], cloud[i][j] ) );
        }
    }
    // i < 15, since: t[15] = 1  !=  t2[15] = 0
    for (size_t i = 0; i < 15; i++) {
        assert( atlaas::allclose( transformation2[i], transformation[i] ) );
    }
    atlaas::c_save("test2.scan", cloud2, length, length2, transformation2);
    free(cloud2);

    std::cout << "done." << std::endl;
    return 0;
}
