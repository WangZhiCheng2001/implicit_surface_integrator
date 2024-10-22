#pragma once

#include <implicit_arrangement.h>
#include <container/hashmap.hpp>

#include "utils/fwd_types.hpp"

struct half_face_t {
    uint32_t index{};
    int8_t   orientation{};
};

using half_face_pair_t = std::pair<half_face_t, half_face_t>;

// compute neighboring pair of half-faces around an iso-edge
// output:
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
void compute_face_order(const IsoEdge                                               &iso_edge,
                        const decltype(tetrahedron_mesh_t::indices)                 &tets,
                        const stl_vector_mp<IsoVertex>                              &iso_verts,
                        const stl_vector_mp<PolygonFace>                            &iso_faces,
                        const stl_vector_mp<Arrangement3D>                          &cut_results,
                        const stl_vector_mp<uint32_t>                               &cut_result_index,
                        const stl_vector_mp<uint32_t>                               &func_in_tet,
                        const stl_vector_mp<uint32_t>                               &start_index_of_tet,
                        const flat_hash_map_mp<uint32_t, small_vector_mp<uint32_t>> &incident_tets,
                        small_vector_mp<half_face_pair_t>                           &ordered_face_pairs);

// compute neighboring pair of half-faces around an iso-edge in a tetrahedron
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
void pair_faces_in_one_tet(const Arrangement3D               &tet_cut_result,
                           const stl_vector_mp<PolygonFace>  &iso_faces,
                           const IsoEdge                     &iso_edge,
                           small_vector_mp<half_face_pair_t> &ordered_face_pairs);

// compute neighboring pair of half-faces around an iso-edge in multiple tetrahedrons
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
void pair_faces_in_tets(const IsoEdge                               &iso_edge,
                        const small_vector_mp<uint32_t>             &containing_simplex,
                        const small_vector_mp<uint32_t>             &containing_tetIds,
                        const decltype(tetrahedron_mesh_t::indices) &tets,
                        const stl_vector_mp<PolygonFace>            &iso_faces,
                        const stl_vector_mp<Arrangement3D>          &cut_results,
                        const stl_vector_mp<uint32_t>               &cut_result_index,
                        const stl_vector_mp<uint32_t>               &func_in_tet,
                        const stl_vector_mp<uint32_t>               &start_index_of_tet,
                        small_vector_mp<half_face_pair_t>           &ordered_face_pairs);