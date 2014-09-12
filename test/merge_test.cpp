#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/atlaas.hpp>
#include <tests.hpp>

int main(int argc, char * argv[]) {
    std::cout << "atlaas merge/slide test..." << std::endl;

    atlaas::atlaas test;
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, true);

    atlaas::points cloud = atlaas::random_cloud();
    atlaas::matrix transformation = atlaas::identity;

    assert( atlaas::allclose( test.get_meta().get_utm_pose_x(), 0.0 ) );
    assert( atlaas::allclose( test.get_meta().get_utm_pose_y(), 0.0 ) );

    test.merge(cloud, transformation);

    assert( atlaas::allclose( test.get_meta().get_utm_pose_x(), -40.0 ) );
    assert( atlaas::allclose( test.get_meta().get_utm_pose_y(),  40.0 ) );

    std::cout << "done." << std::endl;
    return 0;
}
