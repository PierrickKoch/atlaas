/*
 * io.cpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-10-08
 * license: BSD
 */
#include <iostream>
#include <sstream> // ostringstream
#include <atlaas/io.hpp>

#ifdef _USE_PCL
#include <atlaas/pcd.hpp>
#define _CLOUD_SAVE write_pcd
#define _CLOUD_LOAD read_pcd
#define _CLOUD_EXT ".pcd"
#else
#define _CLOUD_SAVE write_raw
#define _CLOUD_LOAD read_raw
#define _CLOUD_EXT ".raw"
#endif

namespace atlaas {

void save(const std::string& filepath,
    const points& cloud, const matrix& transformation) {
      _CLOUD_SAVE(filepath, cloud, transformation);
}
void load(const std::string& filepath,
    points& cloud, matrix& transformation) {
      _CLOUD_LOAD(filepath, cloud, transformation);
}

std::string cloud_filename(size_t seq) {
    std::ostringstream oss;
    oss << "cloud." << std::setfill('0') << std::setw(5) << seq << _CLOUD_EXT;
    return oss.str();
}

} // namespace atlaas
