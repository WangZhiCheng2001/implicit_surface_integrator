#pragma once

#include "evaluation_prev.hpp"

template <typename Routine>
inline auto evaluate(Routine&& tag, const constant_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    if constexpr (is_evaluation_routine_v<Routine>)
        return desc.value;
    else
        return point;
}

template <typename Routine>
inline auto evaluate(Routine&& tag, const plane_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    auto normal   = vec3d_conversion(desc.normal);
    auto distance = normal.dot(point - vec3d_conversion(desc.point));
    if constexpr (is_evaluation_routine_v<Routine>)
        return distance;
    else
        return point - distance * normal;
}

template <typename Routine>
inline auto evaluate(Routine&& tag, const sphere_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    auto center     = vec3d_conversion(desc.center);
    auto p_vec      = point - center;
    auto p_vec_norm = p_vec.norm();
    if constexpr (is_evaluation_routine_v<Routine>)
        return p_vec_norm - desc.radius;
    else
        return center + (p_vec / std::abs(p_vec_norm)) * desc.radius;
}

inline auto evaluate(const cylinder_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    auto        bottom_center = vec3d_conversion(desc.bottom_origion);
    auto        offset        = vec3d_conversion(desc.offset);
    const auto& radius        = desc.radius;

    Eigen::Vector3d ba   = -offset;
    Eigen::Vector3d pa   = point - (bottom_center + offset);
    auto            baba = ba.squaredNorm();
    auto            paba = pa.dot(ba);
    auto            x    = (pa * baba - ba * paba).norm() - radius * baba;
    auto            y    = abs(paba - baba * 0.5) - baba * 0.5;
    auto            x2   = x * x;
    auto            y2   = y * y * baba;
    auto            d    = (std::max(x, y) < 0.0) ? -std::min(x2, y2) : (((x > 0.0) ? x2 : 0.0) + ((y > 0.0) ? y2 : 0.0));
    return sign(d) * std::sqrt(abs(d)) / baba;
}

inline auto evaluate(const cone_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    Eigen::Vector3d ba = vec3d_conversion(desc.bottom_point) - vec3d_conversion(desc.top_point);
    Eigen::Vector3d pa = point - vec3d_conversion(desc.bottom_point);

    auto rba  = desc.radius2 - desc.radius1;
    auto baba = ba.squaredNorm();
    auto paba = pa.dot(ba) / baba;
    auto papa = pa.squaredNorm();
    auto x    = std::sqrt(papa - paba * paba * baba);
    auto cax  = std::max(0.0, x - ((paba < 0.5) ? desc.radius1 : desc.radius2));
    auto cay  = abs(paba - 0.5) - 0.5;
    auto k    = rba * rba + baba;
    auto f    = std::clamp((rba * (x - desc.radius1) + paba * baba) / k, 0.0, 1.0);
    auto cbx  = x - desc.radius1 - f * rba;
    auto cby  = paba - f;
    auto s    = (cbx < 0.0 && cay < 0.0) ? -1.0 : 1.0;
    return s * std::sqrt(std::min(cax * cax + cay * cay * baba, cbx * cbx + cby * cby * baba));
}

template <typename Routine>
inline auto evaluate(Routine&& tag, const box_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    // HINT: this method is not ACCURATE for OUTSIDE OF THE BOX, but it saves time
    auto            center    = vec3d_conversion(desc.center);
    auto            half_size = vec3d_conversion(desc.half_size);
    auto            p_vec     = point - center;
    Eigen::Vector3d d         = p_vec.cwiseAbs() - half_size;

    size_t min_distance_face_mask{};
    auto   distance = d.maxCoeff(&min_distance_face_mask);
    if constexpr (is_evaluation_routine_v<Routine>)
        return distance;
    else {
        if (distance < 0) { // i.e. inside the box
            Eigen::Vector3d result{point};
            std::array      dis_vec{
                center[min_distance_face_mask] - half_size[min_distance_face_mask] - point[min_distance_face_mask],
                center[min_distance_face_mask] + half_size[min_distance_face_mask] - point[min_distance_face_mask],
            };
            result[min_distance_face_mask] += (-dis_vec[0] <= dis_vec[1]) ? dis_vec[0] : dis_vec[1];
            return result;
        } else { // i.e. on surface or outside the box
            return point.cwiseMin(center + half_size).cwiseMax(center - half_size);
        }
    }
}

template <typename Routine>
inline auto evaluate(Routine&& tag, const mesh_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    // Note: There is no check for out-of-bounds access to points, indexes and faces
    auto points  = desc.points;
    auto indices = desc.indices;
    auto face    = desc.faces;

    auto     min_distance{std::numeric_limits<double>::infinity()};
    uint32_t min_distance_face_index{}, min_distance_vert_start_index{};
    uint32_t count{};

    for (auto i = 0; i < desc.face_number; i++) {
        const auto& begin_index = face[i].begin_index;
        const auto& length      = face[i].vertex_count;

        auto point0 = vec3d_conversion(points[indices[begin_index]]);
        bool flag{};
        for (auto j = 1; j < length - 1; j++) {
            auto point1 = vec3d_conversion(points[indices[begin_index + j]]);
            auto point2 = vec3d_conversion(points[indices[begin_index + j + 1]]);
            auto temp   = triangle_sdf(evaluation_tag, point, point0, point1, point2);

            if (temp < min_distance) {
                min_distance                  = temp;
                min_distance_face_index       = i;
                min_distance_vert_start_index = j;
            }

            if (!flag && ray_intersects_triangle(point, x_direction, point0, point1, point2)) { flag = true; }
        }
        if (flag) { count++; }
    }

    if constexpr (is_evaluation_routine_v<Routine>) {
        if (min_distance < 1e-8) { return 0; }
        if (count % 2 == 1) {
            return -min_distance;
        } else {
            return min_distance;
        }
    } else {
        const auto& begin_index = face[min_distance_face_index].begin_index;
        const auto& length      = face[min_distance_face_index].vertex_count;

        auto point0 = vec3d_conversion(points[indices[begin_index]]);
        auto point1 = vec3d_conversion(points[indices[begin_index + min_distance_vert_start_index]]);
        auto point2 = vec3d_conversion(points[indices[begin_index + min_distance_vert_start_index + 1]]);
        return triangle_sdf(closest_point_tag, point, point0, point1, point2);
    }
}