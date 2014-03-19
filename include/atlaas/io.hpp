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

#include <string>

#include <atlaas/common.hpp>

namespace atlaas {

inline void dump(const std::string& filepath,
        const points& cloud, const matrix& transformation) {
    // TODO serialize/archive cloud and transformation into filepath
}

inline void load(const std::string& filepath, points& cloud, matrix& transformation) {
    // TODO parse filepath into cloud and transformation
}

} // namespace atlaas

#endif // ATLAAS_IO_HPP

