#pragma once

#include "primitive_descriptor.h"
#include "solid.hpp"

inline auto raw_to_vec3(const raw_vector3d_t& p) { return Vec3(p.x, p.y, p.z); }

inline auto vec3_to_raw(const Vec3& p) { return raw_vector3d_t{p.x(), p.y(), p.z()}; }

inline auto to_polyline(const polyline_descriptor_t& desc)
{
    Pt3Array points;
    points.resize(desc.point_number);
    for (int i = 0; i < desc.point_number; i++) { points[i] = raw_to_vec3(desc.points[i]); }

    std::vector<double> bugles;
    bugles.resize(desc.bulge_number);
    std::memcpy(bugles.data(), desc.bulges, sizeof(double) * desc.bulge_number);

    return PolyLine{points, bugles, raw_to_vec3(desc.reference_normal), desc.is_close};
}

inline auto to_arcline(const arcline_descriptor_t& desc)
{
    return ArcLine{raw_to_vec3(desc.start), raw_to_vec3(desc.end), desc.bulge, raw_to_vec3(desc.reference_normal)};
}

inline auto to_helixline(const helixline_descriptor_t& desc)
{
    return HelixLine{raw_to_vec3(desc.axis_start),
                     raw_to_vec3(desc.axis_end),
                     desc.radius,
                     desc.advance_per_round,
                     raw_to_vec3(desc.start_direction)};
}