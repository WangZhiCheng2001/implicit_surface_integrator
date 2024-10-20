#include <container/hashmap.hpp>
#include <algorithm/glue_algorithm.hpp>

#include "ia_cut_face.hpp"
#include "robust_assert.hpp"

template <size_t N>
static inline std::array<uint32_t, 3> ia_cut_1_face_impl(IAComplex<N>& ia_complex,
                                                         uint32_t      eid,
                                                         uint32_t      plane_index,
                                                         const int8_t* orientations)
{
    auto& vertices = ia_complex.vertices;
    auto& edges    = ia_complex.edges;
    vertices.reserve(vertices.size() + 1);
    edges.reserve(edges.size() + 2);

    uint32_t positive_subedge_id = INVALID_INDEX;
    uint32_t negative_subedge_id = INVALID_INDEX;
    uint32_t intersection_id     = INVALID_INDEX;

    const auto& e          = edges[eid];
    const auto& end_points = e.vertices;
    const auto  o0         = orientations[end_points[0]];
    const auto  o1         = orientations[end_points[1]];

    intersection_id = end_points[o1 == 0];
    // if (o0 == 0) intersection_id = end_points[0];
    // if (o1 == 0) intersection_id = end_points[1];

    auto compute_intersection_id = [&]() {
        if constexpr (N == 2) {
            auto p0 = e.supporting_plane;
            vertices.push_back({p0, plane_index});
            return static_cast<uint32_t>(vertices.size() - 1);
        } else {
            auto p0 = e.supporting_planes[0];
            auto p1 = e.supporting_planes[1];
            vertices.push_back({p0, p1, plane_index});
            return static_cast<uint32_t>(vertices.size() - 1);
        }
    };

    // do nothing if edge is coplanar with plane (o0 == 0 && o1 == 0)
    if (o0 >= 0 && o1 >= 0) {
        positive_subedge_id = eid;
    } else if (o0 <= 0 && o1 <= 0) {
        negative_subedge_id = eid;
    } else {
        ROBUST_ASSERT(intersection_id == INVALID_INDEX);
        intersection_id = compute_intersection_id();
        IAEdge<N> positive_subedge, negative_subedge;

        if (o0 > 0 && o1 < 0) {
            positive_subedge.vertices = {end_points[0], intersection_id};
            negative_subedge.vertices = {intersection_id, end_points[1]};
        } else {
            ROBUST_ASSERT(o0 < 0);
            ROBUST_ASSERT(o1 > 0);
            negative_subedge.vertices = {end_points[0], intersection_id};
            positive_subedge.vertices = {intersection_id, end_points[1]};
        }

        // Update supporting materials.
        if constexpr (N == 2) {
            positive_subedge.supporting_plane = e.supporting_plane;
            negative_subedge.supporting_plane = e.supporting_plane;
            positive_subedge.positive_face    = e.positive_face;
            positive_subedge.negative_face    = e.negative_face;
            negative_subedge.positive_face    = e.positive_face;
            negative_subedge.negative_face    = e.negative_face;
        } else {
            positive_subedge.supporting_planes = e.supporting_planes;
            negative_subedge.supporting_planes = e.supporting_planes;
        }

        edges.emplace_back(std::move(positive_subedge));
        edges.emplace_back(std::move(negative_subedge));
        positive_subedge_id = static_cast<uint32_t>(edges.size() - 2);
        negative_subedge_id = static_cast<uint32_t>(edges.size() - 1);
    }
    return {positive_subedge_id, negative_subedge_id, intersection_id};
}

std::array<uint32_t, 3> ia_cut_1_face(IAComplex<2>& ia_complex, uint32_t eid, uint32_t plane_index, const int8_t* orientations)
{
    return ia_cut_1_face_impl(ia_complex, eid, plane_index, orientations);
}

std::array<uint32_t, 3> ia_cut_1_face(IAComplex<3>& ia_complex, uint32_t eid, uint32_t plane_index, const int8_t* orientations)
{
    return ia_cut_1_face_impl(ia_complex, eid, plane_index, orientations);
}