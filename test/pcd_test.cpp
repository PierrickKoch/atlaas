#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/pcd.hpp>

int main(int argc, char * argv[]) {
    int N = 200000;
    std::cout << "atlaas pcd test..." << std::endl;

    atlaas::matrix tio, transformation = {{
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1,
    }};
    atlaas::points cio, cloud;
    cloud.resize(N);
    for(uint i = 0; i < cloud.size(); i++){
        for(uint j = 0; j < cloud[i].size(); j++){
            float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            cloud[i][j] = 1000*r;
        }
    }

    atlaas::write_pcd("test.pcd", cloud, transformation);
    atlaas::write_pcd_voxel("test_voxel.pcd", cloud, transformation);
    atlaas::read_pcd("test.pcd", cio, tio);


    // assert(cloud == c2);
    // assert(transformation == t2);

    std::cout << "done." << std::endl;
    return 0;
}
