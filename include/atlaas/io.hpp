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
#include <cstdlib> // std::malloc

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

/**
* save from raw C array (using std::copy !)
* used for numpy -> C++ interface
* transformation must be double[16] : row-major Matrix(4,4)
* cloud must be float[cloud_len1][cloud_len2]
* cloud_len2 must be either 3 (XYZ) or 4 (XYZI)
*/
inline void c_save(const std::string& filepath, const float* cloud,
        size_t cloud_len1, size_t cloud_len2, const double* transformation) {
    matrix tr;
    points cd( cloud_len1 );
    std::copy(transformation, transformation + 16, tr.begin());
    for (size_t i = 0; i < cloud_len1; i++)
        std::copy(cloud+i*cloud_len2, cloud+(i+1)*cloud_len2, cd[i].begin());
    save(filepath, cd, tr);
}

inline float* c_load(const std::string& filepath, size_t& length,
        double* transformation) {
    matrix tr;
    points cd;
    load(filepath, cd, tr);
    length = cd.size();
    float* cloud = (float*) std::malloc(length*4*sizeof(float));
    for (size_t i = 0; i < length; i++)
        std::copy(cd[i].begin(), cd[i].end(), &cloud[i*4]);
    std::copy(tr.begin(), tr.end(), transformation);
    return cloud;
}

} // namespace atlaas

#endif // ATLAAS_IO_HPP
