#pragma once

#include "plane.hpp"
#include "ia_structure.hpp"

int8_t ia_cut_0_face(const PlaneGroup2D& planes, const IAComplex<2>& ia_complex, uint32_t vid, uint32_t plane_index);
int8_t ia_cut_0_face(const PlaneGroup3D& planes, const IAComplex<3>& ia_complex, uint32_t vid, uint32_t plane_index);
std::array<uint32_t, 3> ia_cut_1_face(IAComplex<2>& ia_complex, uint32_t eid, uint32_t plane_index, const int8_t* orientations);
std::array<uint32_t, 3> ia_cut_1_face(IAComplex<3>& ia_complex, uint32_t eid, uint32_t plane_index, const int8_t* orientations);
std::array<uint32_t, 3> ia_cut_2_face(IAComplex<2>&                  ia_complex,
                                      uint32_t                       fid,
                                      uint32_t                       plane_index,
                                      const int8_t*                  orientations,
                                      const std::array<uint32_t, 3>* subedges);
std::array<uint32_t, 3> ia_cut_2_face(IAComplex<3>&                  ia_complex,
                                      uint32_t                       fid,
                                      uint32_t                       plane_index,
                                      const int8_t*                  orientations,
                                      const std::array<uint32_t, 3>* subedges);
std::array<uint32_t, 3> ia_cut_3_face(IAComplex<3>&                  ia_complex,
                                      uint32_t                       cid,
                                      uint32_t                       plane_index,
                                      const std::array<uint32_t, 3>* subfaces);