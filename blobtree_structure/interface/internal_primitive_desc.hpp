#pragma once

#include <vector>

#include <tbb/tbb.h>
#include <utils/eigen_alias.hpp>

#include "primitive_descriptor.h"
#include "internal_structs.hpp"

namespace internal
{
template <typename T>
using stl_vector = std::vector<T, tbb::tbb_allocator<T>>;

using constant = constant_descriptor_t;
using plane    = plane_descriptor_t;
using sphere   = sphere_descriptor_t;
using cone     = cone_descriptor_t;

struct mesh {
    stl_vector<double>               vertices{};
    stl_vector<stl_vector<uint32_t>> faces{};
};

// CAUTION: in polyline local space, the X/Y axis should be according to local N/B directions
struct polyline {
    std::vector<Eigen::Vector2d> vertices{}; //
                                             // for a straight line, it consists of two vertices; for a circle arc, it consists
                                             // of three vertices: start point, circle center, and a point on the circle to
                                             // construct local XY coordinate system
    std::vector<double>          thetas{};   // for a straight line, this should be 0
    std::vector<uint8_t>         start_indices{}; // has the same size as thetas, indicating the starting index of each segment
};

struct helixline {
    // Eigen::Vector3d start_point{};
    // Eigen::Vector3d axis_direction{};
    // Eigen::Vector3d base_direction_u{};
    // Eigen::Vector3d base_direction_v{};
    double radius{};
    double total_theta{};
    double height{};
};

struct extrude_polyline {
    Eigen::Transform<double, 3, Eigen::AffineCompact> world_to_axis{};
    polyline                                          axis{};
    polyline                                          profile{};
};

struct extrude_helixline {
    Eigen::Transform<double, 3, Eigen::AffineCompact> world_to_axis{};
    helixline                                         axis{};
    polyline                                          profile{};
};
} // namespace internal