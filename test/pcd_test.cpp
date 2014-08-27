#undef NDEBUG
#include <cassert>
#include <iostream>
#include <atlaas/atlaas.hpp>

int main(int argc, char * argv[]) {
    std::cout << "atlaas pcd test..." << std::endl;

    atlaas::atlaas test;

    atlaas::matrix tio, transformation = {{
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1,
    }};
    atlaas::points cio, cloud = {{
        {{ 1, 2, 3}},
        {{-1,-2,-3}},
    }};

    test.write_pcd("test.pcd", cloud, transformation);
    test.read_pcd("test.pcd", cio, tio);

    using namespace atlaas;
    std::cout<<cloud<<std::endl;
    std::cout<<cio<<std::endl;
    std::cout<<transformation<<std::endl;
    std::cout<<tio<<std::endl;

    // assert(cloud == c2);
    // assert(transformation == t2);

    std::cout << "done." << std::endl;
    return 0;
}
