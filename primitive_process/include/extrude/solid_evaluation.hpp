#pragma once

#include "line_evaluation.hpp"

namespace internal
{

// =============================================================
// structures
// =============================================================

struct extrude_closest_param_t {
    Eigen::Vector3d point{};
    double          distance{std::numeric_limits<double>::max()};
};

// Eigen has a type Isometry which supports similar operations as this
// but we use AffineCompact to get lower storage cost
// so we have to implement a helper function to apply the operation of Isometry to AffineCompact
static inline auto inversed_affine_transform(const Eigen::Transform<double, 3, Eigen::AffineCompact>& trs)
{
    Eigen::Transform<double, 3, Eigen::AffineCompact> result;

    auto linear_part                                = result.matrix().template topLeftCorner<3, 3>();
    linear_part                                     = trs.linear().transpose();
    result.matrix().template topRightCorner<3, 1>() = -linear_part * trs.translation();
    result.makeAffine();

    return result;
}

// =============================================================
// derived functions
// =============================================================

// CAUTION: take care of relations between line's local TBN and plane's local TBN
// CAUTION: all internal calculations of polyline should assume that p is in axis/plane's local TBN space
static inline auto get_local_TBN(const polyline&                          line,
                                 const Eigen::Ref<const Eigen::Vector3d>& p,
                                 const Eigen::Ref<const Eigen::Vector3d>& q,
                                 double                                   t)
{
    Eigen::Vector2d local_tangent, local_normal;
    if (t >= 0 && t <= line.start_indices.size() && t - std::round(t) <= EPSILON) {
        // if between two segments, use local vec{pq} on the plane as normal
        local_normal = (q.topRows<2>() - p.topRows<2>()).normalized();
        if (local_normal.dot(calculate_normal(line, t)) < 0) local_normal *= -1;
        local_tangent = {local_normal[1], -local_normal[0]};
    } else {
        local_tangent = calculate_tangent(line, t);
        local_normal  = calculate_normal(line, t);
    }

    Eigen::Transform<double, 3, Eigen::Isometry> TBN{};
    auto&                                        handle = TBN.matrix();
    handle.col(0).topRows<2>()                          = std::move(local_normal);
    handle.col(1)[2]                                    = 1;
    handle.col(2).topRows<2>()                          = std::move(local_tangent);

    return TBN;
}

static inline auto get_local_TBN(const helixline& line, double t)
{
    Eigen::Transform<double, 3, Eigen::Isometry> TBN{};
    auto&                                        handle = TBN.matrix();
    const auto                                   T      = calculate_tangent(line, t);
    const auto                                   N      = calculate_normal(line, t);
    const auto                                   B      = T.cross(N);
    handle.col(0)                                       = std::move(N);
    handle.col(1)                                       = std::move(B);
    handle.col(2)                                       = std::move(T);

    return TBN;
}

[[nodiscard]] static inline auto calculate_closest_param(const extrude_helixline& solid,
                                                         const Eigen::Vector3d&   p) -> extrude_closest_param_t
{
    const auto local_p            = solid.world_to_axis * p;
    const auto axis_closest_param = calculate_closest_param(solid.axis, local_p);
    const auto TBN                = get_local_TBN(solid.axis, axis_closest_param.t);
    const auto inv_TBN            = TBN.matrix().transpose();

    // TODO: add support for non-parallel (ray vs. profile plane) case
    // for now just assume that profile plane is parallel to axis
    const auto vec_qp                = local_p - axis_closest_param.point;
    const auto vec_qp_proj           = inv_TBN * vec_qp;
    const auto profile_closest_param = calculate_closest_param(solid.profile, vec_qp_proj);

    // transform closest point to world space
    // the distance should be unchanged during transformation, since no scaling is involved
    const auto world_closest_point =
        inversed_affine_transform(solid.world_to_axis) * (TBN * axis_closest_param.point + axis_closest_param.point);
    return {std::move(world_closest_point), profile_closest_param.distance};
}

} // namespace internal