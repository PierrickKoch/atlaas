#include <iostream>
#include <pcl/point_cloud.h>

int main(int argc, char * argv[]) {
    std::array<double, 16>
        t1 = {{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1, }},
        t2 = {{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1, }};

    // IN
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> e1((double*)t1.data());

    Eigen::Quaternionf q1 = Eigen::Quaternionf( e1.topLeftCorner<3,3>().cast<float>() );
    Eigen::Vector4f v1 = Eigen::Vector4f( e1.topRightCorner<4,1>().cast<float>() );

    // OUT
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> e2((double*)t2.data());

    e2.topLeftCorner<3,3>() = q1.toRotationMatrix().cast<double>();
    e2.topRightCorner<4,1>() = v1.cast<double>();

    // DISP
    std::cout<<e1<<std::endl;
    std::cout<<e2<<std::endl;
    return 0;
}
