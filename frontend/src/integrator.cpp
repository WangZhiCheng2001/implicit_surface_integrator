#include <background_mesh.hpp>
#include <load_functions.hpp>

#include <integrator.hpp>

setting_descriptor        g_settings{};
ImplicitSurfaceIntegrator g_integrator{};

void ImplicitSurfaceIntegrator::update_background_mesh(const Eigen::Ref<const raw_point_t>& aabb_min,
                                                       const Eigen::Ref<const raw_point_t>& aabb_max) noexcept
{
    assert(g_settings.resolution > 0);

    this->background_mesh = std::move(generate_tetrahedron_background_mesh(g_settings.resolution, aabb_min, aabb_max));
}

void ImplicitSurfaceIntegrator::update_scene(const char* file) noexcept
{
    auto implicit_functions = load_functions(file);
    if (!implicit_functions.success) { std::cerr << "Failed to load implicit functions from file." << std::endl; }

    this->sdf_scalar_field.resize(implicit_functions.functions.size(), this->background_mesh.vertices.size());
    for (size_t i = 0; i < this->background_mesh.vertices.size(); ++i) {
        const auto& vertex = background_mesh.vertices[i];
        for (size_t j = 0; j < implicit_functions.functions.size(); ++j) {
            std::visit([&](auto& function) { this->sdf_scalar_field(j, i) = function.evaluate_scalar(vertex); },
                       implicit_functions.functions[j]);
        }
    }
}

void ImplicitSurfaceIntegrator::clear() noexcept
{
    iso_vertices.clear();
    iso_faces.clear();
    patches.clear();
    patch_function_labels.clear();
    iso_edges.clear();
    non_manifold_edges_of_vert.clear();
    shells.clear();
    arrangement_cells.clear();
    cell_function_labels.clear();
    m_per_face_surf_int.clear();
    m_per_face_vol_int.clear();
}

void ImplicitSurfaceIntegrator::compute(labelled_timers_manager& timers_manager) noexcept
{
    timers_manager.push_timer("compute surf & vol integral");

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

    timers_manager.pop_timer("compute surf & vol integral");
}