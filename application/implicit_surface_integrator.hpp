#pragma once

#include <implicit_surface_network_processor.hpp>

class ImplicitSurfaceIntegrator : public ImplicitSurfaceNetworkProcessor
{
public:
    using ImplicitSurfaceNetworkProcessor::ImplicitSurfaceNetworkProcessor;

    void clear_results()
    {
        m_per_face_surf_int.clear();
        m_per_face_vol_int.clear();
    }

    void compute()
    {
        m_per_face_surf_int.reserve(iso_faces.size());
        m_per_face_vol_int.reserve(iso_faces.size());

        Eigen::Vector3d temp_area_vector{};
        for (const auto& face : iso_faces) {
            const auto&     vertices = face.vertex_indices;
            const auto&     v0       = iso_vertices[vertices[0]];
            double          surf_int{};
            Eigen::Vector3d area_vector_sum{};
            for (auto iter = vertices.begin() + 2; iter != vertices.end(); ++iter) {
                const auto &v1 = iso_vertices[*(iter - 1)], v2 = iso_vertices[*iter];
                temp_area_vector  = (v1 - v0).cross(v2 - v0);
                area_vector_sum  += temp_area_vector;
                surf_int         += temp_area_vector.norm();
            }
            m_per_face_surf_int.emplace_back(surf_int / 2.0);
            m_per_face_vol_int.emplace_back(area_vector_sum.dot(v0) / 6.0);
        }
    }

private:
    stl_vector_mp<double> m_per_face_surf_int{};
    stl_vector_mp<double> m_per_face_vol_int{};
};