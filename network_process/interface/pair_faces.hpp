#pragma once

#include <container/hashmap.hpp>

#include <utils/fwd_types.hpp>

// compute neighboring pair of half-patches around an iso-edge
// output:
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
ISNP_API void compute_patch_order(const iso_edge_t                                                   &iso_edge,
                                  const stl_vector_mp<tetrahedron_vertex_indices_t>                  &tets,
                                  const stl_vector_mp<iso_vertex_t>                                  &iso_verts,
                                  const stl_vector_mp<polygon_face_t>                                &iso_faces,
                                  const stl_vector_mp<std::shared_ptr<arrangement_t>>                &cut_results,
                                  const stl_vector_mp<uint32_t>                                      &func_in_tet,
                                  const stl_vector_mp<uint32_t>                                      &start_index_of_tet,
                                  const parallel_flat_hash_map_mp<uint32_t, stl_vector_mp<uint32_t>> &incident_tets,
                                  const stl_vector_mp<uint32_t>                                      &patch_of_face_mapping,
                                  stl_vector_mp<half_patch_pair_t>                                   &ordered_patch_pairs);

// compute neighboring pair of half-patches around an iso-edge in a tetrahedron
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
ISNP_API void pair_patches_in_one_tet(const arrangement_t                 &tet_cut_result,
                                      const stl_vector_mp<polygon_face_t> &iso_faces,
                                      const iso_edge_t                    &iso_edge,
                                      const stl_vector_mp<uint32_t>       &patch_of_face_mapping,
                                      stl_vector_mp<half_patch_pair_t>    &ordered_patch_pairs);

// compute neighboring pair of half-patches around an iso-edge in multiple tetrahedrons
// pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
ISNP_API void pair_patches_in_tets(const iso_edge_t                                    &iso_edge,
                                   const stl_vector_mp<uint32_t>                       &containing_simplex,
                                   const stl_vector_mp<uint32_t>                       &containing_tetIds,
                                   const stl_vector_mp<tetrahedron_vertex_indices_t>   &tets,
                                   const stl_vector_mp<polygon_face_t>                 &iso_faces,
                                   const stl_vector_mp<std::shared_ptr<arrangement_t>> &cut_results,
                                   const stl_vector_mp<uint32_t>                       &func_in_tet,
                                   const stl_vector_mp<uint32_t>                       &start_index_of_tet,
                                   const stl_vector_mp<uint32_t>                       &patch_of_face_mapping,
                                   stl_vector_mp<half_patch_pair_t>                    &ordered_patch_pairs);