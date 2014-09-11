/*
 * pcd.cpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-10-08
 * license: BSD
 */
#include <iostream>
#include <atlaas/pcd.hpp>

#ifdef _USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#endif

namespace atlaas {

#ifdef _USE_PCL

void read_pcd(const std::string& filepath, points& cloud,
        matrix& transformation) {
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::io::loadPCDFile(filepath, input);
    cloud.resize( input.points.size() );
    auto it = cloud.begin();
    for (const auto& point : input.points) {
        (*it)[0] = point.x;
        (*it)[1] = point.y;
        (*it)[2] = point.z;
        (*it)[3] = point.intensity;
        ++it;
    }
    transformation = {{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1, }}; // identity
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
        eigen_matrix((double*)transformation.data());
    eigen_matrix.topLeftCorner<3,3>() =
        input.sensor_orientation_.toRotationMatrix().cast<double>();
    eigen_matrix.topRightCorner<4,1>() =
        input.sensor_origin_.cast<double>();
}

void write_pcd(const std::string& filepath, const points& cloud,
        const matrix& transformation) {
    pcl::PointCloud<pcl::PointXYZI> output;
    output.height = 1;
    output.width = cloud.size();
    output.is_dense = true;
    output.points.resize( cloud.size() );
    auto it = output.points.begin();
    for (const auto& point : cloud) {
        (*it).x = point[0];
        (*it).y = point[1];
        (*it).z = point[2];
        (*it).intensity = point[3];
        ++it;
    }
    // set transformation sensor-world
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
        eigen_matrix((double*)transformation.data());
    output.sensor_orientation_ =
        Eigen::Quaternionf( eigen_matrix.topLeftCorner<3,3>().cast<float>() );
    output.sensor_origin_ =
        Eigen::Vector4f( eigen_matrix.topRightCorner<4,1>().cast<float>() );
    // save pcd
    pcl::io::savePCDFileBinaryCompressed(filepath, output);
}

void write_pcd_voxel(const std::string& filepath, const points& cloud,
        const matrix& transformation, float voxel_size, float dist_sq) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> output;
    pcloud->height = 1;
    pcloud->is_dense = true;
    pcloud->points.resize( cloud.size() );
    auto it = pcloud->points.begin();
    for (const auto& point : cloud) {
        if (length_sq(point) < dist_sq) {
            (*it).x = point[0];
            (*it).y = point[1];
            (*it).z = point[2];
            (*it).intensity = point[3];
            ++it;
        }
    }
    /* Removes the elements in the range [it,end] */
    pcloud->points.erase(it, pcloud->points.end());
    pcloud->width = pcloud->points.size();
    // set transformation sensor-world
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
        eigen_matrix((double*)transformation.data());
    pcloud->sensor_orientation_ =
        Eigen::Quaternionf( eigen_matrix.topLeftCorner<3,3>().cast<float>() );
    pcloud->sensor_origin_ =
        Eigen::Vector4f( eigen_matrix.topRightCorner<4,1>().cast<float>() );
    // voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setInputCloud ( pcloud );
    grid.setLeafSize (voxel_size, voxel_size, voxel_size);
    grid.filter (output);
    // save pcd
    pcl::io::savePCDFileBinaryCompressed(filepath, output);
}

#else

void read_pcd(const std::string& filepath, points& cloud,
        matrix& transformation) {
    std::cerr<<"[warn] atlaas was not compiled with PCL: "<<__func__<<std::endl;
}
void write_pcd(const std::string& filepath, const points& cloud,
        const matrix& transformation) {
    std::cerr<<"[warn] atlaas was not compiled with PCL: "<<__func__<<std::endl;
}
void write_pcd_voxel(const std::string& filepath, const points& cloud,
        const matrix& transformation, float voxel_size, float dist_sq) {
    std::cerr<<"[warn] atlaas was not compiled with PCL: "<<__func__<<std::endl;
}

#endif

} // namespace atlaas
