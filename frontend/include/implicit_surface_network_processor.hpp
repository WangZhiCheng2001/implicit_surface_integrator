#pragma once

#include <implicit_arrangement.hpp>

#include "background_mesh_manager.hpp"
#include "patch_integrator.hpp"
#include "patch_propagator.hpp"

struct ImplicitSurfaceNetworkProcessor {
    void           preinit(const virtual_node_t& tree_node) noexcept;
    void           clear() noexcept;
    solve_result_t run(const virtual_node_t& tree_node) noexcept;

    /* adaptors */
    BackgroundMeshManager background_mesh_manager{};
    PatchIntegrator       patch_integrator{};
    PatchPropagator       patch_propagator{};

    /* intermediate */
    stl_vector_mp<uint32_t> leaf_index_of_primitive{};

    /* output fields */
    // topology results
    stl_vector_mp<raw_point_t> iso_vertices{}; ///< Vertices at the surface network mesh
    stl_vector_mp<uint32_t>    polygon_faces{};
    stl_vector_mp<uint32_t>    vertex_counts_of_face{};
};