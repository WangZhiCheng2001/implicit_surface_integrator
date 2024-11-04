#pragma once

#include <implicit_arrangement.hpp>
#include <timer/scoped_timer.hpp>

#include "utils/fwd_types.hpp"

struct ISNP_API ImplicitSurfaceNetworkProcessor {
    bool run(labelled_timers_manager& timers_manager);

    /* input fields */
    tetrahedron_mesh_t background_mesh{};
    Eigen::MatrixXd    sdf_scalar_field{}; ///< 2D matrix of function values at all vertices of the background mesh

    /* output fields */
    stl_vector_mp<raw_point_t>             iso_vertices{}; ///< Vertices at the surface network mesh
    stl_vector_mp<polygon_face_t>          iso_faces{};    ///< Polygonal faces at the surface network mesh
    stl_vector_mp<stl_vector_mp<uint32_t>> patches{};      ///< A connected component of faces bounded by non-manifold edges
    stl_vector_mp<uint32_t>   patch_function_labels{};     ///< index of the zero-valued function that creates the patch
    stl_vector_mp<iso_edge_t> iso_edges{};                 ///< Edges at the surface network mesh
    stl_vector_mp<stl_vector_mp<uint32_t>> chains{};       ///< Chains of non-manifold edges
    stl_vector_mp<stl_vector_mp<uint32_t>> non_manifold_edges_of_vert{}; ///< Indices of non-manifold vertices
    stl_vector_mp<stl_vector_mp<uint32_t>>
        shells{}; ///< An array of shells. Each shell is a connected component consist of patches. Even patch index, 2*i,
                  ///< indicates patch i is consistently oriented with the shell. Odd patch index, 2*i+1, indicates patch i has
                  ///< opposite orientation with respect to the shell.
    stl_vector_mp<stl_vector_mp<uint32_t>>
        arrangement_cells{};    ///< A 3D region partitioned by the surface network; encoded by a vector of shell indices
    stl_vector_mp<stl_vector_mp<bool>>
        cell_function_labels{}; ///< A 2D boolean array for the signs of each pair of a function and an arrangement cell
};