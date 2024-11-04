#include <container/hashmap.hpp>
#include <algorithm/glue_algorithm.hpp>

#include "ia_cut_face.hpp"
#include "implicit_arrangement.hpp"
#include "robust_assert.hpp"

std::array<uint32_t, 3> ia_cut_2_face(ia_complex_t&                                 ia_complex,
                                      uint32_t                                      fid,
                                      uint32_t                                      plane_index,
                                      const stl_vector_mp<int8_t>&                  orientations,
                                      const stl_vector_mp<std::array<uint32_t, 3>>& subedges)
{
    auto& edges = ia_complex.edges;
    auto& faces = ia_complex.faces;
    edges.reserve(edges.size() + 1);
    faces.reserve(faces.size() + 2);

    auto&        f                  = faces[fid];
    const size_t num_boundary_edges = f.edges.size();

    stl_vector_mp<uint32_t> positive_subedges{}, negative_subedges{};
    positive_subedges.reserve(num_boundary_edges);
    negative_subedges.reserve(num_boundary_edges);

    ia_edge_t cut_edge;
    uint32_t  cut_edge_index             = INVALID_INDEX;
    bool      face_is_coplanar           = true;
    uint32_t  cut_edge_positive_location = INVALID_INDEX;
    uint32_t  cut_edge_negative_location = INVALID_INDEX;

    auto get_end_vertex = [&](uint32_t local_eid) {
        auto        curr_eid = f.edges[local_eid];
        auto        next_eid = f.edges[(local_eid + 1) % num_boundary_edges];
        const auto& e0       = edges[curr_eid];
        const auto& e1       = edges[next_eid];
        if (e0.vertices[0] == e1.vertices[0] || e0.vertices[0] == e1.vertices[1]) {
            return e0.vertices[0];
        } else {
            ROBUST_ASSERT(e0.vertices[1] == e1.vertices[0] || e0.vertices[1] == e1.vertices[1]);
            return e0.vertices[1];
        }
    };

    for (size_t j = 0; j < num_boundary_edges; j++) {
        const auto eid = f.edges[j];

        bool last_positive      = false;
        bool last_negative      = false;
        auto intersection_point = subedges[eid][2];
        auto positive_subedge   = subedges[eid][0];
        auto negative_subedge   = subedges[eid][1];

        if (positive_subedge == INVALID_INDEX && negative_subedge == INVALID_INDEX) {
            // Edge is coplanar with the plane.
            cut_edge_index = eid;
        } else {
            if (positive_subedge != INVALID_INDEX) {
                positive_subedges.emplace_back(positive_subedge);
                auto end_vid = get_end_vertex(static_cast<uint32_t>(j));
                if (orientations[end_vid] <= 0) {
                    // This edge is the last edge with positive subedge.
                    cut_edge_positive_location = positive_subedges.size();
                    last_positive              = true;
                }
            }
            if (negative_subedge != INVALID_INDEX) {
                negative_subedges.emplace_back(negative_subedge);
                auto end_vid = get_end_vertex(static_cast<uint32_t>(j));
                if (orientations[end_vid] >= 0) {
                    // This edge is the last edge with negative subedge.
                    cut_edge_negative_location = negative_subedges.size();
                    last_negative              = true;
                }
            }
            face_is_coplanar = false;
        }
        if (intersection_point != INVALID_INDEX) {
            if (last_positive) {
                cut_edge.vertices[0] = intersection_point;
            } else if (last_negative) {
                cut_edge.vertices[1] = intersection_point;
            }
        }
    }

    if (face_is_coplanar) { return {INVALID_INDEX, INVALID_INDEX, INVALID_INDEX}; }

    if (positive_subedges.empty() || negative_subedges.empty()) {
        // No cut.
        if (positive_subedges.empty()) {
            ROBUST_ASSERT(!negative_subedges.empty());
            return {INVALID_INDEX, fid, cut_edge_index};
        } else {
            ROBUST_ASSERT(!positive_subedges.empty());
            ROBUST_ASSERT(negative_subedges.empty());
            return {fid, INVALID_INDEX, cut_edge_index};
        }
    }

    ROBUST_ASSERT(cut_edge_index == INVALID_INDEX);
    {
        // Insert cut edge.
        cut_edge.supporting_planes = {f.supporting_plane, plane_index};
        cut_edge_index             = edges.size();
        edges.emplace_back(std::move(cut_edge));
    }

    // Create subfaces.
    ia_face_t positive_subface, negative_subface;
    positive_subface.supporting_plane = f.supporting_plane;
    negative_subface.supporting_plane = f.supporting_plane;
    positive_subface.positive_cell    = f.positive_cell;
    positive_subface.negative_cell    = f.negative_cell;
    negative_subface.positive_cell    = f.positive_cell;
    negative_subface.negative_cell    = f.negative_cell;

    ROBUST_ASSERT(cut_edge_index != INVALID_INDEX);
    if (cut_edge_positive_location != positive_subedges.size()) {
        algorithm::rotate<algorithm::ExecutionPolicySelector::simd_only>(positive_subedges.begin(),
                                                                         positive_subedges.begin() + cut_edge_positive_location,
                                                                         positive_subedges.end());
    }
    if (cut_edge_negative_location != negative_subedges.size()) {
        algorithm::rotate<algorithm::ExecutionPolicySelector::simd_only>(negative_subedges.begin(),
                                                                         negative_subedges.begin() + cut_edge_negative_location,
                                                                         negative_subedges.end());
    }
    positive_subedges.emplace_back(cut_edge_index);
    positive_subface.edges = std::move(positive_subedges);
    ROBUST_ASSERT(positive_subface.edges.size() > 2);
    negative_subedges.emplace_back(cut_edge_index);
    negative_subface.edges = std::move(negative_subedges);
    ROBUST_ASSERT(negative_subface.edges.size() > 2);

    faces.emplace_back(std::move(positive_subface));
    faces.emplace_back(std::move(negative_subface));
    uint32_t positive_fid = static_cast<uint32_t>(faces.size() - 2);
    uint32_t negative_fid = static_cast<uint32_t>(faces.size() - 1);

    return {positive_fid, negative_fid, cut_edge_index};
}