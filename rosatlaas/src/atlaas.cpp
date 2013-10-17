#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <atlaas/atlaas.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

atlaas::atlaas al;
sensor_msgs::PointCloud2 cloud;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PCDWriter writer;
    writer.write ("/tmp/atlaas.pcd", input, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
}

void cloud_grid_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    // ... do data processing
    // http://pointclouds.org/documentation/tutorials/#filtering-tutorial
    // sensor_msgs::PointCloud2 cloud;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(cloud);
    // al.merge(cloud.points);
    // write to disk
    pcl::PCDWriter writer;
    writer.write ("/tmp/atlaas.pcd", cloud, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "atlaas");
    ros::NodeHandle nh;
    // Init a terrain of 500x500m at 2m res. (w/o spec UTM meta)
    al.init(500, 500, 2, 0, 0, 0, 0, 31);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
    // Spin
    ros::spin();
    // Save map
    al.get().save("/tmp/atlaas.tif");
    return 0;
}

// ./bin/atlaas input:=/robot/velodyne
