#pragma once

#include "utils/fwd_types.h"

tetrahedron_mesh_t generate_tetrahedron_background_mesh(size_t             resolution,
                                                        const raw_point_t& aabb_min,
                                                        const raw_point_t& aabb_max);