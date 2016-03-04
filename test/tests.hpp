/*
 * tests.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2014-09-12
 * license: BSD
 */
#ifndef TESTS_HPP
#define TESTS_HPP

#include <cstdlib> // std::rand
#include <type_traits>
#include <limits>
#include <atlaas/common.hpp>

namespace atlaas {

template < class T,
           class = typename std::enable_if<std::is_scalar<T>::value>::type >
bool allclose(T a, T b, T eps = std::numeric_limits<T>::epsilon()) {
    return (std::abs(a-b) <= eps);
}

template <typename T, size_t N>
bool allclose(const std::array<T, N>& a, const std::array<T, N>& b) {
    for (size_t i = 0; i < N; i++)
        if (! allclose(a[i], b[i]) )
            return false;
    return true;
}

template <typename T>
bool allclose(const std::vector<T>& a, const std::vector<T>& b) {
    if ( a.size() != b.size() )
        return false;
    for (size_t i = 0; i < a.size(); i++)
        if (! allclose(a[i], b[i]) )
            return false;
    return true;
}

matrix identity = {{
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1,
}};

points random_cloud(size_t size = 200000, double scale = 1000.0) {
    points cloud(size);
    scale /= RAND_MAX;
    for (size_t i = 0; i < cloud.size(); i++) {
        for (size_t j = 0; j < cloud[i].size(); j++) {
            cloud[i][j] = scale * std::rand();
        }
    }
    return cloud;
}

} // namespace atlaas

#endif // TESTS_HPP
