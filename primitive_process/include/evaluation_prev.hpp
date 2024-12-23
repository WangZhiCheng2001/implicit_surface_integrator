#pragma once

#include <utils/eigen_alias.hpp>

#include <primitive_descriptor.h>

// =========================================================================================================================

struct evaluation_routine_tag {
};

struct closest_point_routine_tag {
};

template <typename T>
static constexpr bool is_process_routine_tag_v = false;

template <>
static constexpr bool is_process_routine_tag_v<evaluation_routine_tag> = true;

template <>
static constexpr bool is_process_routine_tag_v<closest_point_routine_tag> = true;

template <typename T>
static constexpr bool is_evaluation_routine_v = std::is_same_v<T, evaluation_routine_tag>;

template <typename T>
static constexpr bool is_closest_point_routine_v = std::is_same_v<T, closest_point_routine_tag>;

static constexpr evaluation_routine_tag    evaluation_tag{};
static constexpr closest_point_routine_tag closest_point_tag{};

// =========================================================================================================================

inline auto vec3d_conversion(const raw_vector3d_t& p) { return Eigen::Map<const Eigen::Vector3d>(&p.x); }

inline double sign(const double t) { return t >= 0.0 ? 1.0 : -1.0; }

template <typename Routine, typename = std::enable_if_t<is_process_routine_tag_v<Routine>>>
inline auto triangle_sdf(Routine&&                                tag,
                         const Eigen::Ref<const Eigen::Vector3d>& p,
                         const Eigen::Ref<const Eigen::Vector3d>& a,
                         const Eigen::Ref<const Eigen::Vector3d>& b,
                         const Eigen::Ref<const Eigen::Vector3d>& c)
{
    auto ba  = b - a;
    auto pa  = p - a;
    auto cb  = c - b;
    auto pb  = p - b;
    auto ac  = a - c;
    auto pc  = p - c;
    auto nor = ba.cross(ac);

    Eigen::Vector3d test_vals = {sign(pa.dot(ba.cross(nor))), //
                                 sign(pb.dot(cb.cross(nor))), //
                                 sign(pc.dot(ac.cross(nor)))};
    if (test_vals.sum() < 2.0) {
        std::array closest_points = {a + ba * std::clamp(ba.dot(pa) / ba.squaredNorm(), 0.0, 1.0),
                                     b + cb * std::clamp(cb.dot(pb) / cb.squaredNorm(), 0.0, 1.0),
                                     c + ac * std::clamp(ac.dot(pc) / ac.squaredNorm(), 0.0, 1.0)};
        std::array distance = {(closest_points[0] - p).norm(), (closest_points[1] - p).norm(), (closest_points[2] - p).norm()};
        auto       min_iter = std::min_element(distance.begin(), distance.end());
        if constexpr (is_evaluation_routine_v<Routine>)
            return *min_iter;
        else
            return closest_points[std::distance(distance.begin(), min_iter)];
    } else {
        auto distance = pa.dot(nor) / nor.norm();
        if constexpr (is_evaluation_routine_v<Routine>)
            return std::abs(distance);
        else
            return p - distance * nor.normalized();
    }
}

inline bool ray_intersects_triangle(const Eigen::Ref<const Eigen::Vector3d>& point,
                                    const Eigen::Ref<const Eigen::Vector3d>& dir,
                                    const Eigen::Ref<const Eigen::Vector3d>& v0,
                                    const Eigen::Ref<const Eigen::Vector3d>& v1,
                                    const Eigen::Ref<const Eigen::Vector3d>& v2)
{
    auto e1    = v1 - v0;
    auto e2    = v2 - v0;
    auto s     = point - v0;
    auto s1    = dir.cross(e2);
    auto s2    = s.cross(e1);
    auto coeff = 1.0 / s1.dot(e1);
    auto t     = coeff * s2.dot(e2);
    auto b1    = coeff * s1.dot(s);
    auto b2    = coeff * s2.dot(dir);
    return t >= 0 && b1 >= 0 && b2 >= 0 && (1 - b1 - b2) >= 0;
}

static const auto x_direction = Eigen::Vector3d{1.0, 0.0, 0.0};