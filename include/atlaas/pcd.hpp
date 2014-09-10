/*
 * pcd.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-10-08
 * license: BSD
 */
#ifndef ATLAAS_PCD_HPP
#define ATLAAS_PCD_HPP

#include <string>
#include <atlaas/common.hpp>

#define PCD_VOXEL_SIZE 0.05f
#define PCD_DIST_SQ 400.0f

namespace atlaas {

void read_pcd(const std::string& filepath, points& cloud,
        matrix& transformation);
void write_pcd(const std::string& filepath, const points& cloud,
        const matrix& transformation);
void write_pcd_voxel(const std::string& filepath, const points& cloud,
        const matrix& transformation, float voxel_size = PCD_VOXEL_SIZE,
        float dist_sq = PCD_DIST_SQ);

} // namespace atlaas

#endif // ATLAAS_PCD_HPP
