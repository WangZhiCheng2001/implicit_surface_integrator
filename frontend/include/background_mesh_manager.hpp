#pragma once

#include <background_mesh.hpp>

class BackgroundMeshManager
{
public:
    void generate(const Eigen::Ref<const raw_point_t>& aabb_min, const Eigen::Ref<const raw_point_t>& aabb_max) noexcept;

    const auto get_vertices() const noexcept { return m_background_mesh.vertices; }

    const auto get_indices() const noexcept { return m_background_mesh.indices; }

    const auto identity() const noexcept { return m_background_mesh; }

private:
    tetrahedron_mesh_t m_background_mesh{};
};