#pragma once

#include "plane.hpp"
#include "ia_structure.hpp"

int8_t ia_cut_0_face(const plane_group_t& planes, const ia_complex_t& ia_complex, uint32_t vid, uint32_t plane_index);
std::array<uint32_t, 3> ia_cut_1_face(ia_complex_t&                ia_complex,
                                      uint32_t                     eid,
                                      uint32_t                     plane_index,
                                      const stl_vector_mp<int8_t>& orientations);
std::array<uint32_t, 3> ia_cut_2_face(ia_complex_t&                                 ia_complex,
                                      uint32_t                                      fid,
                                      uint32_t                                      plane_index,
                                      const stl_vector_mp<int8_t>&                  orientations,
                                      const stl_vector_mp<std::array<uint32_t, 3>>& subedges);
std::array<uint32_t, 3> ia_cut_3_face(ia_complex_t&                                 ia_complex,
                                      uint32_t                                      cid,
                                      uint32_t                                      plane_index,
                                      const stl_vector_mp<std::array<uint32_t, 3>>& subfaces);