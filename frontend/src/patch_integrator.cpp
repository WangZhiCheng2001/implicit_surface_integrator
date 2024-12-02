#include "patch_integrator.hpp"

std::pair<double, double> PatchIntegrator::integrate(const stl_vector_mp<raw_point_t>&    vertices,
                                                     const stl_vector_mp<polygon_face_t>& faces,
                                                     const stl_vector_mp<uint32_t>&       face_of_patch_mapping) noexcept
{
    std::pair<double, double> result{}; // (surface_integral, volume_integral)

    Eigen::Vector3d temp_area_vector{};
    for (const auto& face_id : face_of_patch_mapping) {
        const auto& face       = faces[face_id];
        const auto& vertex_ids = face.vertex_indices;

        // not a polygon face, skip it
        if (vertex_ids.size() < 3) continue;

        const auto&     v0 = vertices[vertex_ids[0]];
        Eigen::Vector3d area_vector_sum{};
        for (auto iter = vertex_ids.begin() + 2; iter != vertex_ids.end(); ++iter) {
            const auto &v1 = vertices[*(iter - 1)], v2 = vertices[*iter];
            temp_area_vector  = (v1 - v0).cross(v2 - v0);
            area_vector_sum  += temp_area_vector;
            result.first     += temp_area_vector.norm() * 0.5;
        }
        result.second += v0.dot(area_vector_sum) / 6;
    }

    return result;
}