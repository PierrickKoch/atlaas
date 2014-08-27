/*
 * atlaas.cpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-10-08
 * license: BSD
 */
#include <cassert>

#include <atlaas/atlaas.hpp>

#ifdef _USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#endif

namespace atlaas {

#ifdef _USE_PCL

void atlaas::read_pcd(const std::string& filepath, points& cloud, matrix& transformation) {
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZI> input;
    reader.read(filepath, input);
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
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> eigen_matrix((double*)transformation.data());
    eigen_matrix.topLeftCorner<3,3>() = input.sensor_orientation_.toRotationMatrix().cast<double>();
    eigen_matrix.topRightCorner<4,1>() = input.sensor_origin_.cast<double>();
}

void atlaas::write_pcd(const std::string& filepath, const points& cloud, const matrix& transformation) {
    typedef pcl::PointCloud<pcl::PointXYZI> pc_t;
    pc_t::Ptr pcloud(new pc_t);
    pcloud->height = 1;
    pcloud->is_dense = true;
    pcloud->points.resize( cloud.size() );
    auto it = pcloud->points.begin();
    for (const auto& point : cloud) {
        if (length_sq(point) < 400) {
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
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> eigen_matrix((double*)transformation.data());
    pcloud->sensor_orientation_ = Eigen::Quaternionf( eigen_matrix.topLeftCorner<3,3>().cast<float>() );
    pcloud->sensor_origin_ = Eigen::Vector4f( eigen_matrix.topRightCorner<4,1>().cast<float>() );
    // voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setInputCloud (pcloud);
    grid.setLeafSize (0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZI> output;
    grid.filter (output);
    // save pcd
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(filepath, output);
}

void atlaas::write_pcd(const points& cloud, const matrix& transformation) {
    write_pcd(pcdpath(pcd_seq++), cloud, transformation);
}

#else

void atlaas::write_pcd(const points& cloud, const matrix& transformation) {}
void atlaas::write_pcd(const std::string& filepath, const points& cloud, const matrix& transformation) {}
void atlaas::read_pcd(const std::string& filepath, points& cloud, matrix& transformation) {}

#endif

} // namespace atlaas
