#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/atlaas.hpp>


void assert_io_cloud(atlaas::points &cloud, atlaas::points &cloud2){
    assert( cloud.size() == cloud2.size());
    for(uint i = 0; i < cloud.size(); i++){
        for(uint j = 0; j < cloud[i].size(); j++){
            assert( cloud[i][j] == cloud2[i][j]);
        }
    }
}

void assert_io_transformation( atlaas::matrix &transformation,  atlaas::matrix &transformation2){
    for(uint i = 0; i < transformation.size(); i++){
        assert( transformation[i] == transformation2[i]);
    }
}

int main(int argc, char * argv[]) {
    std::cout << "atlaas io test..." << std::endl;
    int N = 200000;
    // atlaas::atlaas test;
    // test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, true);
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

    std::cout << "generate " << N << " random points" << std::endl;
    cloud.resize(N);
    for(uint i = 0; i < cloud.size(); i++){
        for(uint j = 0; j < cloud[i].size(); j++){
            float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            cloud[i][j] = 1000*r;
        }
    }

    std::string filename = "test.scan";
    std::cout << " dump scan into file " << filename << std::endl;

    atlaas::dump(filename, cloud, transformation);

    atlaas::matrix transformation2;
    atlaas::points cloud2;

    std::cout << " read scan from file " << filename << std::endl;

    atlaas::load(filename, cloud2, transformation2);

    assert_io_cloud(cloud, cloud2);
    assert_io_transformation(transformation,transformation2);

    std::cout << "done." << std::endl;
    return 0;
}
