#pragma once

#include "primitive_descriptor.h"

#include "utils/eigen_alias.hpp"

struct aabb_t {
    Eigen::Vector3d min{std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max()};
    Eigen::Vector3d max{std::numeric_limits<double>::min(),
                        std::numeric_limits<double>::min(),
                        std::numeric_limits<double>::min()};

    void extend(const Eigen::Vector3d& point)
    {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    void extend(const aabb_t& aabb)
    {
        min = min.cwiseMin(aabb.min);
        max = max.cwiseMax(aabb.max);
    }

    void offset(const Eigen::Vector3d& offset)
    {
        min = min + offset;
        max = max + offset;
    }

};