#include <container/hashmap.hpp>
#include <algorithm/glue_algorithm.hpp>
#include <implicit_predicates.hpp>

#include "ia_cut_face.hpp"
#include "robust_assert.hpp"

inline int8_t signof(orientation o)
{
    ROBUST_ASSERT(o != orientation::invalid);
    return static_cast<int8_t>(o);
}

#ifdef LUT_GENERATE
// extern std::array<std::array<bool, 3>, 4> vertex_signs;
// extern std::array<bool, 6>                edge_signs;

// template <size_t N>
// static inline int8_t ia_cut_0_face_impl(const PlaneGroup<N>& planes,
//                                         const IAComplex<N>&  ia_complex,
//                                         uint32_t             vid,
//                                         uint32_t             plane_index)
// {
//     ROBUST_ASSERT(plane_index == 4 || plane_index == 5);
//     const auto& v = ia_complex.vertices[vid];

//     if (v.i0 < 4 && v.i1 < 4 && v.i2 < 4) {
//         // Case 1: v is a tet vertex.
//         for (size_t i = 0; i < 4; i++) {
//             if (i != v.i0 && i != v.i1 && i != v.i2) { return vertex_signs[i][plane_index - 4] ? 1 : -1; }
//         }
//         ROBUST_ASSERT(false);
//     }

//     // Case 2: v is on a tet edge.
//     ROBUST_ASSERT(plane_index == 5);
//     auto edge_key = INVALID_INDEX;
//     auto P1 = INVALID_INDEX, P2 = INVALID_INDEX;
//     for (size_t i = 0; i < 3; i++) {
//         if ((&v.i0)[i] < 4) {
//             if (P1 == INVALID_INDEX) {
//                 P1 = (&v.i0)[i];
//             } else {
//                 P2 = (&v.i0)[i];
//             }
//         }
//     }
//     if (P1 > P2) std::swap(P1, P2);

//     if (P1 == 0 && P2 == 1) {
//         edge_key = 5;
//     } else if (P1 == 0 && P2 == 2) {
//         edge_key = 4;
//     } else if (P1 == 0 && P2 == 3) {
//         edge_key = 3;
//     } else if (P1 == 1 && P2 == 2) {
//         edge_key = 2;
//     } else if (P1 == 1 && P2 == 3) {
//         edge_key = 1;
//     } else if (P1 == 2 && P2 == 3) {
//         edge_key = 0;
//     }

//     return edge_signs[edge_key] ? 1 : -1;
// }
#else

int8_t ia_cut_0_face(const plane_group_t& planes, const ia_complex_t& ia_complex, uint32_t vid, uint32_t plane_index)
{
    const auto& p  = planes.get_plane(plane_index);
    const auto& v  = ia_complex.vertices[vid];
    const auto& p0 = planes.get_plane(v[0]);
    const auto& p1 = planes.get_plane(v[1]);

    const auto& p2 = planes.get_plane(v[2]);
    return signof(orient3d(p0.data(), p1.data(), p2.data(), p.data()));
}

#endif