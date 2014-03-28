/*
 * matcher.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2014-03-31
 * license: BSD
 */
#ifndef ATLAAS_ICP_HPP
#define ATLAAS_ICP_HPP

#include <atlaas/common.hpp>
#include <pointmatcher/PointMatcher.h>

namespace atlaas {

/**
 * pointmatcher (ICP)
 */
class matcher {
    // transform
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Parameters Parameters;
    PM::ICP icp;
    DP ref;
    // convert
    typedef typename PM::DataPoints::Label  Label;
    typedef typename PM::DataPoints::Labels Labels;
    typedef typename PM::Matrix Matrix;

    bool first;
    points convert(const DP& dp) {
        int i = 0;
        points out( dp.features.cols() );
        for (auto& point : out) {
            point[0] = dp.features(0, i);
            point[1] = dp.features(1, i);
            point[2] = dp.features(2, i);
            i++;
        }
        return out;
    }
    DP convert(const points& in) {
        int i = 0;
        Labels featureLabels;
        featureLabels.push_back(Label("x", 1));
        featureLabels.push_back(Label("y", 1));
        featureLabels.push_back(Label("z", 1));
        featureLabels.push_back(Label("pad", 1));
        Matrix features( 4, in.size() );
        for (const auto& point : in) {
            features(0, i) = point[0];
            features(1, i) = point[1];
            features(2, i) = point[2];
            features(3, i) = 1;
            i++;
        }
        return PM::DataPoints(features, featureLabels);
    }
public:
    matcher() {
        icp.setDefault();
        first = true;
    }
    points transform(const points& cloud) {
        DP data = convert(cloud);
        // if not ref: ref = data; return cloud;
        if (first) {
            PM::swapDataPoints(ref, data);
            first = false;
            std::cout << "pm::icp first" << std::endl;
            return cloud;
        }
        // Compute the transformation to express data in ref
        PM::TransformationParameters T = icp(data, ref);
        // Transform data to express it in ref
        icp.transformations.apply(data, T);

        std::cout << "pm::icp transformation:" << std::endl << T << std::endl;
        PM::swapDataPoints(ref, data);
        return convert(ref);
    }
};

} // namespace atlaas

#endif // ATLAAS_ICP_HPP

