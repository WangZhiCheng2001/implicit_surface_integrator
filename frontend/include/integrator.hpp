#pragma once

#include <implicit_surface_network_processor.hpp>

#include <environment.h>

/* global singleton */
extern setting_descriptor g_settings;

class ImplicitSurfaceIntegrator : public ImplicitSurfaceNetworkProcessor
{
public:
    // ImplicitSurfaceIntegrator(const tetrahedron_mesh_t&                background_mesh,
    //                           const Eigen::Ref<const Eigen::MatrixXd>& sdf_scalar_field)
    //     : background_mesh(background_mesh)
    // {
    //     this->sdf_scalar_field(sdf_scalar_field);
    // }

    void update_background_mesh(const Eigen::Ref<const raw_point_t>& aabb_min,
                                const Eigen::Ref<const raw_point_t>& aabb_max) noexcept;
    void update_scene(const char* file) noexcept;
    void clear() noexcept;
    void compute(labelled_timers_manager& timers_manager) noexcept;

private:
    stl_vector_mp<double> m_per_face_surf_int{};
    stl_vector_mp<double> m_per_face_vol_int{};
};

/* global singleton */
extern ImplicitSurfaceIntegrator g_integrator;