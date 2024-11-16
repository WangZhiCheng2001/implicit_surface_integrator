#include "globals.hpp"
#include "background_mesh_manager.hpp"

void BackgroundMeshManager::generate(const Eigen::Ref<const raw_point_t>& aabb_min,
                                     const Eigen::Ref<const raw_point_t>& aabb_max) noexcept
{
    assert(g_settings.resolution > 0);

    this->m_background_mesh = std::move(generate_tetrahedron_background_mesh(g_settings.resolution, aabb_min, aabb_max));
}