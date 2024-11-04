#pragma once

#include <utils/fwd_types.hpp>

ISNP_API tetrahedron_mesh_t generate_tetrahedron_background_mesh(uint32_t                             resolution,
                                                                 const Eigen::Ref<const raw_point_t>& aabb_min,
                                                                 const Eigen::Ref<const raw_point_t>& aabb_max);