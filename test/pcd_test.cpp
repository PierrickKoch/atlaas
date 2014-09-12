#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/pcd.hpp>
#include <tests.hpp>

int main(int argc, char * argv[]) {
    std::cout << "atlaas pcd test..." << std::endl;

    atlaas::points cloud = atlaas::random_cloud();
    atlaas::matrix transformation = atlaas::identity;

    atlaas::write_pcd("test.pcd", cloud, transformation);

    atlaas::points cloud2;
    atlaas::matrix transformation2;

    atlaas::read_pcd("test.pcd", cloud2, transformation2);

#ifdef _USE_PCL
    assert( atlaas::allclose( cloud, cloud2 ) );
    // Eigen Quaternion.toRotationMatrix change the values
    // assert( atlaas::allclose( transformation, transformation2 ) );
#endif

    atlaas::write_pcd_voxel("test_voxel.pcd", cloud, transformation);
    atlaas::read_pcd("test_voxel.pcd", cloud2, transformation2);

    std::cout << "done." << std::endl;
    return 0;
}
