/*
 * io.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2014-03-19
 * license: BSD
 */
#ifndef ATLAAS_IO_HPP
#define ATLAAS_IO_HPP

#include <string> // std::string
#include <iostream> // std::ios
#include <fstream> // {i,o}fstream
#include <cassert> // assert
#include <stdexcept> // std::runtime_error

#include <atlaas/common.hpp>

namespace atlaas {

/*
 * Dump a point cloud and its transformation matrix
 * into a file <filename>
 */
template<typename PointT>
inline void write_raw(const std::string& filepath,
    const std::vector<PointT>& cloud, const matrix& transformation) {

    std::ofstream file(filepath, std::ios::out | std::ios::trunc | std::ios::binary);
    if ( file.fail() )
        throw std::runtime_error("could not open file");

    // write point size
    size_t point_size = sizeof(PointT);
    file.write((char*)&point_size, sizeof(size_t));
    // write transformation
    file.write((char*)&transformation[0], sizeof(matrix));
    // write cloud size
    size_t cloud_size = cloud.size();
    file.write((char*)&cloud_size, sizeof(size_t));
    // write cloud
    file.write((char*)&cloud[0], sizeof(PointT)*cloud.size());
}

template<typename PointT>
inline void read_raw(const std::string& filepath,
    std::vector<PointT>& cloud, matrix& transformation) {

    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    if ( file.fail() )
        throw std::runtime_error("could not open file");

    // read point size
    size_t point_size;
    file.read(reinterpret_cast<char*>(&point_size), sizeof(size_t));
    assert(point_size == sizeof(PointT));
    // read transformation
    file.read((char*)&transformation[0], sizeof(matrix));
    // read cloud size
    size_t cloud_size;
    file.read((char*)&cloud_size, sizeof(size_t));
    cloud.resize(cloud_size);
    // read cloud
    file.read((char*)&cloud[0], cloud_size * sizeof(PointT));
}

void save(const std::string& filepath,
    const points& cloud, const matrix& transformation);
void load(const std::string& filepath,
    points& cloud, matrix& transformation);
std::string cloud_filename(size_t seq);

} // namespace atlaas

#endif // ATLAAS_IO_HPP
