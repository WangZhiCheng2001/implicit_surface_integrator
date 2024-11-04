#pragma once

#include <container/hashmap.hpp>

#include <utils/fwd_types.hpp>

struct half_face_t {
    uint32_t index{};
    int8_t   orientation{};
};

using half_face_pair_t = std::pair<half_face_t, half_face_t>;

// compute neighboring pair of half-faces around an iso-edge
// output:
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
ISNP_API void compute_face_order(const iso_edge_t                                          &iso_edge,
                                 const stl_vector_mp<tetrahedron_vertex_indices_t>         &tets,
                                 const stl_vector_mp<iso_vertex_t>                         &iso_verts,
                                 const stl_vector_mp<polygon_face_t>                       &iso_faces,
                                 const stl_vector_mp<arrangement_t>                        &cut_results,
                                 const stl_vector_mp<uint32_t>                             &cut_result_index,
                                 const stl_vector_mp<uint32_t>                             &func_in_tet,
                                 const stl_vector_mp<uint32_t>                             &start_index_of_tet,
                                 const flat_hash_map_mp<uint32_t, stl_vector_mp<uint32_t>> &incident_tets,
                                 stl_vector_mp<half_face_pair_t>                           &ordered_face_pairs);

// compute neighboring pair of half-faces around an iso-edge in a tetrahedron
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
ISNP_API void pair_faces_in_one_tet(const arrangement_t                 &tet_cut_result,
                                    const stl_vector_mp<polygon_face_t> &iso_faces,
                                    const iso_edge_t                    &iso_edge,
                                    stl_vector_mp<half_face_pair_t>     &ordered_face_pairs);

// compute neighboring pair of half-faces around an iso-edge in multiple tetrahedrons
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
ISNP_API void pair_faces_in_tets(const iso_edge_t                                  &iso_edge,
                                 const stl_vector_mp<uint32_t>                     &containing_simplex,
                                 const stl_vector_mp<uint32_t>                     &containing_tetIds,
                                 const stl_vector_mp<tetrahedron_vertex_indices_t> &tets,
                                 const stl_vector_mp<polygon_face_t>               &iso_faces,
                                 const stl_vector_mp<arrangement_t>                &cut_results,
                                 const stl_vector_mp<uint32_t>                     &cut_result_index,
                                 const stl_vector_mp<uint32_t>                     &func_in_tet,
                                 const stl_vector_mp<uint32_t>                     &start_index_of_tet,
                                 stl_vector_mp<half_face_pair_t>                   &ordered_face_pairs);