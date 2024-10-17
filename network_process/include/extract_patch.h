#pragma once

#include <utils/fwd_types.h>

// extract iso-mesh (topology only)
void extract_iso_mesh(size_t                                                     num_1_func,
                      size_t                                                     num_2_func,
                      size_t                                                     num_more_func,
                      const std::vector<simplicial_arrangement::Arrangement<3>>& cut_results,
                      const std::vector<size_t>&                                 cut_result_index,
                      const std::vector<size_t>&                                 func_in_tet,
                      const std::vector<size_t>&                                 start_index_of_tet,
                      const std::vector<std::array<size_t, 4>>&                  tets,
                      std::vector<IsoVert>&                                      iso_verts,
                      std::vector<PolygonFace>&                                  iso_faces);

// extract iso-mesh (topology only) and create map: local index --> global index
void extract_iso_mesh(size_t                                                     num_1_func,
                      size_t                                                     num_2_func,
                      size_t                                                     num_more_func,
                      const std::vector<simplicial_arrangement::Arrangement<3>>& cut_results,
                      const std::vector<size_t>&                                 cut_result_index,
                      const std::vector<size_t>&                                 func_in_tet,
                      const std::vector<size_t>&                                 start_index_of_tet,
                      const std::vector<std::array<size_t, 4>>&                  tets,
                      std::vector<IsoVert>&                                      iso_verts,
                      std::vector<PolygonFace>&                                  iso_faces,
                      std::vector<long long>&                                    global_vId_of_tet_vert,
                      std::vector<size_t>&                                       global_vId_start_index_of_tet,
                      std::vector<size_t>&                                       iso_fId_of_tet_face,
                      std::vector<size_t>&                                       iso_fId_start_index_of_tet);