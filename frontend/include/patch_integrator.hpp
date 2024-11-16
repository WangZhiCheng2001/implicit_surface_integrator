#pragma once

#include <utils/fwd_types.hpp>

class PatchIntegrator
{
public:
    std::pair<double, double> integrate(const stl_vector_mp<raw_point_t>&    vertices,
                                        const stl_vector_mp<polygon_face_t>& faces,
                                        const stl_vector_mp<uint32_t>&       face_of_patch_mapping) noexcept;
};